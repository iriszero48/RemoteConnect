// #if defined(_DEBUG) || defined(DEBUG)
// #define ASIO_ENABLE_HANDLER_TRACKING
// #endif

#include <random>

#include <asio/detached.hpp>
#include <asio/co_spawn.hpp>
#include <asio/io_context.hpp>
#include <asio/ip/tcp.hpp>
#include <asio/signal_set.hpp>
#include <asio/awaitable.hpp>
#include <asio/redirect_error.hpp>
#include <asio/use_awaitable.hpp>
#include <asio/write.hpp>
#include <asio/read.hpp>

#include <nlohmann/json.hpp>

#include "Arguments/Arguments.hpp"
#include "Log/Log.hpp"
#include "String/String.hpp"
#include "StdIO/StdIO.hpp"
#include "Time/Time.hpp"
#include "Utility/Utility.hpp"
#include "Video/Video.hpp"
#include "Assert/Assert.hpp"

#if WIN32
#include <windows.h>
#else
#include <X11/Xlib.h>
#endif

#include "RcUtility.hpp"

#ifdef ASIO_ENABLE_HANDLER_TRACKING
#define RcUtil_UseAwaitable asio::use_awaitable_t(__FILE__, __LINE__, __FUNCTION__)
#else
#define RcUtil_UseAwaitable asio::use_awaitable
#endif

#ifdef None
#define Rc_None 0L
#undef None
#endif

[[nodiscard]] inline auto GetScreenWH()
{
#if WIN32
    const auto width = GetSystemMetrics(SM_CXSCREEN);
    const auto height = GetSystemMetrics(SM_CYSCREEN);
    return std::make_tuple(width, height);
#else
    Display *disp = XOpenDisplay(NULL);
    Screen *scrn = DefaultScreenOfDisplay(disp);
    const auto width = scrn->width;
    const auto height = scrn->height;
    return std::make_tuple(width, height);
#endif
}

#pragma region LogDefs
struct LogMsg
{
    decltype(std::chrono::system_clock::now()) Time;
    std::string Source;
    std::u8string Message;

    static decltype(auto) LogTime(const decltype(Time) &time)
    {
        auto t = std::chrono::system_clock::to_time_t(time);
        tm local{};
        CuTime::Local(&local, &t);
        std::ostringstream ss;
        ss << std::put_time(&local, "%F %X");
        return ss.str();
    }
};

static CuLog::Logger<LogMsg> Log{};

#define Log__Impl(level, ...)                                                                                                \
    if (level <= Log.Level)                                                                                                  \
    Log.Write<level>(                                                                                                        \
        std::chrono::system_clock::now(),                                                                                    \
        std::string(CuUtil::String::Combine("[", CuUtil_Filename, ":", CuUtil_LineString "] [", __FUNCTION__, "] ").data()), \
        CuStr::FormatU8(__VA_ARGS__))
#define LogNone(...) Log__Impl(CuLog::LogLevel::None, __VA_ARGS__)
#define LogErr(...) Log__Impl(CuLog::LogLevel::Error, __VA_ARGS__)
#define LogWarn(...) Log__Impl(CuLog::LogLevel::Warn, __VA_ARGS__)
#define LogInfo(...) Log__Impl(CuLog::LogLevel::Info, __VA_ARGS__)
#define LogVerb(...) Log__Impl(CuLog::LogLevel::Verb, __VA_ARGS__)
#define LogDebug(...) Log__Impl(CuLog::LogLevel::Debug, __VA_ARGS__)
#pragma endregion LogDefs

namespace RcUtil
{
    template <typename T, typename AsyncWriteStream>
    asio::awaitable<void> WriteInt(AsyncWriteStream &sock, T value)
    {
        if constexpr (std::endian::native == std::endian::big)
        {
            value = CuBit::ByteSwap(value);
        }

        std::error_code ec;
        co_await asio::async_write(sock, asio::buffer(&value, sizeof(T)), redirect_error(RcUtil_UseAwaitable, ec));
        if (ec)
            throw MakeApiExcept("asio::async_write", "{}", ec.message());
        LogDebug("write: {}", value);

        co_return;
    }

    template <typename T, typename AsyncWriteStream>
    asio::awaitable<void> WriteString(AsyncWriteStream &sock, const std::string_view &str)
    {
        CuAssert(str.length() <= std::numeric_limits<T>::max());

        const auto len = static_cast<T>(str.length());
        co_await WriteInt(sock, len);

        std::error_code ec;
        co_await asio::async_write(sock, asio::buffer(str), redirect_error(RcUtil_UseAwaitable, ec));
        if (ec)
            throw MakeApiExcept("asio::async_write", "{}", ec.message());
        LogDebug("write: {}", str);

        co_return;
    }

#define RcUtil__MakeWriteString(suffix, type)                                                      \
    template <typename AsyncWriteStream>                                                           \
    asio::awaitable<void> WriteString##suffix(AsyncWriteStream &sock, const std::string_view &str) \
    {                                                                                              \
        co_await WriteString<type>(sock, str);                                                     \
        co_return;                                                                                 \
    }

    RcUtil__MakeWriteString(2, uint16_t);

    template <typename T, typename AsyncReadStream>
    asio::awaitable<T> ReadInt(AsyncReadStream &sock)
    {
        std::array<uint8_t, sizeof(T)> buffer{};
        std::error_code ec;
        co_await async_read(sock, asio::buffer(buffer), redirect_error(RcUtil_UseAwaitable, ec));
        if (ec)
            throw MakeApiExcept("asio::async_read", "{}", ec.message());

        auto *val = reinterpret_cast<T *>(buffer.data());
        if constexpr (std::endian::native == std::endian::big)
        {
            *val = CuBit::ByteSwap(*val);
        }

        LogDebug("read: {}", *val);
        co_return *val;
    }

    template <typename T, typename AsyncReadStream>
    asio::awaitable<std::string> ReadString(AsyncReadStream &sock)
    {
        const auto len = co_await ReadInt<T>(sock);

        std::error_code ec;
        std::string pw(len, 0);
        co_await asio::async_read(sock, asio::buffer(pw, len), redirect_error(RcUtil_UseAwaitable, ec));
        if (ec)
            throw MakeApiExcept("asio::async_read", "{}", ec.message());

        LogDebug("read: {}", pw);
        co_return pw;
    }

#define RcUtil__MakeReadString(suffix, type)                               \
    template <typename AsyncReadStream>                                    \
    asio::awaitable<std::string> ReadString##suffix(AsyncReadStream &sock) \
    {                                                                      \
        auto str = co_await ReadString<type>(sock);                        \
        co_return str;                                                     \
    }

    RcUtil__MakeReadString(1, uint8_t);
    RcUtil__MakeReadString(2, uint16_t);
}

static struct ServerLaunchConfig
{
    std::string Password{};

    std::string Hostname = "localhost";
    uint16_t ServerPort = 8080;
    uint16_t VideoPort = 8081;

    std::string UrlHostname = Hostname;
    uint16_t UrlPort = VideoPort;

    CuLog::LogLevel LogLevelValue = CuLog::LogLevel::Debug;

    std::optional<int32_t> Width = {};
    std::optional<int32_t> Height = {};
    std::optional<int32_t> WidthOffset = {};
    std::optional<int32_t> HeightOffset = {};
} LaunchConfig;

class IClient
{
public:
    IClient() = default;
    virtual ~IClient() = default;

    IClient(const IClient &) = default;
    IClient(IClient &&) = default;
    IClient &operator=(const IClient &) = default;
    IClient &operator=(IClient &&) = default;

    virtual void Exit() = 0;
};

class VideoServer
{
    std::jthread bgThread{};
    std::atomic_bool isRunning = false;
    std::atomic<std::shared_ptr<const std::u8string>> currentUrl = nullptr;
    std::atomic_uint64_t currentId{};

    std::random_device rd{};
    std::uniform_int_distribution<uint64_t> ud{};

public:
    VideoServer()
    {
        auto level = AV_LOG_TRACE;

        switch (Log.Level)
        {
        case CuLog::LogLevel::None:
            level = AV_LOG_QUIET;
            break;
        case CuLog::LogLevel::Error:
            level = AV_LOG_ERROR;
            break;
        case CuLog::LogLevel::Warn:
            level = AV_LOG_WARNING;
            break;
        case CuLog::LogLevel::Info:
            level = AV_LOG_INFO;
            break;
        case CuLog::LogLevel::Verb:
            level = AV_LOG_VERBOSE;
            break;
        case CuLog::LogLevel::Debug:
            level = AV_LOG_VERBOSE;
            break;
        }

        av_log_set_level(level);
    }

    const std::u8string &GetCurrentUrl() const
    {
        return *currentUrl.load();
    }

    std::optional<std::string> Start()
    {
        if (isRunning)
            return "running";

        currentId = ud(rd);
        currentUrl = std::make_shared<std::u8string>(CuStr::FormatU8("rtmp://{}:{}/rc/{}", LaunchConfig.UrlHostname, LaunchConfig.UrlPort, currentId.load()));

        bgThread = std::jthread([&](const std::stop_token &tk)
                                {
        	isRunning = true;

            try
            {
                CuVid::DecoderRGB capturer;
                CuVid::EncoderRGB streamer;

#ifdef _MSC_VER
                capturer.Config.Input = "desktop";
                capturer.Config.FormatName = u8"gdigrab";
                capturer.Config.Options
                    .Set(u8"framerate", 15)
                    .Set(u8"video_size", u8"1920x1080")
                    .Set(u8"offset_x", 0)
                    .Set(u8"offset_y", 0);
#else
                // ffmpeg -video_size 1024x768 -framerate 25 -f x11grab -i :0.0+100,200 output.mp4
                capturer.Config.Input = ":0.0+0,0";
                capturer.Config.FormatName = u8"x11grab";
                capturer.Config.Options
                    .Set(u8"framerate", 15)
                    .Set(u8"video_size", u8"1920x1080");
#endif

                streamer.Config.OutputPath = CuStr::FormatU8("rtmp://{}:{}/rc/{}", LaunchConfig.Hostname, LaunchConfig.VideoPort, currentId.load());
                streamer.Config.FormatName = u8"flv";
                streamer.Config.Video.CodecIdOrName = u8"h264_nvenc";
                streamer.Config.Video.TimeBase = { 1, 15 };
                streamer.Config.Video.Width = 1920;
                streamer.Config.Video.Height = 1080;
                streamer.Config.Opt.Set(u8"listen", 1);

                streamer.Init();

                capturer.Config.VideoHandler = [&, i = int64_t{ 0 }](const auto& frame) mutable
                {
	                streamer.Write(CuImg::ConvertToConstRef(frame), i++);
                };

                capturer.LoadFile();
                capturer.FindStream();

                while (!capturer.Eof() && !tk.stop_requested())
                {
                    capturer.Read();
                }

                streamer.Finish();
            }
            catch (const std::exception& ex)
            {
                LogErr(ex.what());
            }

            currentId = {};
            currentUrl = std::make_shared<std::u8string>();
            isRunning = false; });

        return {};
    }

    void Exit()
    {
        bgThread.request_stop();
    }
};

class Client : IClient, public std::enable_shared_from_this<Client>
{
    using OperatorBuffer = std::array<char, 4>;

    asio::ip::tcp::socket clientSocket;
    VideoServer &videoServer;

public:
    Client(asio::ip::tcp::socket socket, VideoServer &vidSrv) : clientSocket(std::move(socket)), videoServer(vidSrv)
    {
        LogDebug("accept client");
    }

    asio::awaitable<void> Start()
    {
        try
        {
            LogDebug("start socket: {}:{}", clientSocket.remote_endpoint().address(), clientSocket.remote_endpoint().port());

            if (co_await ReadOperator() != ServerOperator::Auth)
                throw MakeExcept("expect an Auth");

            if (const auto pw = co_await ReadPassword(); pw != LaunchConfig.Password)
            {
                LogErr("Auth failed");
                co_await RcUtil::WriteString2(clientSocket, nlohmann::json{{"status", false}, {"error", "password error"}}.dump());
                co_return;
            }

            videoServer.Start();
            co_await RcUtil::WriteString2(clientSocket, nlohmann::json{{"status", true}, {"result", CuStr::ToDirtyUtf8StringView(videoServer.GetCurrentUrl())}}.dump());

            co_spawn(
                clientSocket.get_executor(), [self = shared_from_this()]
                { return self->Reader(); },
                asio::detached);
            // asio::co_spawn(clientSocket.get_executor(), [self = shared_from_this()] { return self->Writer(); }, asio::detached);
        }
        catch (const std::exception &e)
        {
            LogErr("start failed: {}", e.what());
        }
    }

    void Exit() override
    {
        LogDebug("exit client");
    }

private:
#ifdef CuUtil_Platform_Windows
    const std::unordered_map<RcKey, decltype(VK_LWIN)> keyMap = []
    {
        std::unordered_map<RcKey, decltype(VK_LWIN)> buf{};

        buf[RcKey::Key_Meta] = VK_LWIN;

        const auto mapRange = [&](const RcKey begin, const RcKey end, decltype(VK_LWIN) vcBegin)
        {
            for (auto i = CuUtil::ToUnderlying(begin); i != CuUtil::ToUnderlying(end); ++i, ++vcBegin)
            {
                buf[static_cast<RcKey>(i)] = vcBegin;
            }
        };

        mapRange(RcKey::Key_0, RcKey::Key_Colon, '0');
        mapRange(RcKey::Key_A, RcKey::Key_BracketLeft, 'A');
        mapRange(RcKey::Key_F1, RcKey::Key_F13, VK_F1);

        return buf;
    }();
#endif

    // read op
    asio::awaitable<void> Reader()
    {
        bool running = true;
        while (running)
        {
            switch (co_await ReadOperator())
            {
            case ServerOperator::Exit:
                videoServer.Exit();
                running = false;
                break;
            case ServerOperator::MsMv:
            {
                const auto data = nlohmann::json::parse(co_await RcUtil::ReadString2(clientSocket));
                const auto x = data["x"].get<double>();
                const auto y = data["y"].get<double>();

#ifdef CuUtil_Platform_Windows
                SetCursorPos(x, y);
#else
                Display *dpy = XOpenDisplay(0);
                int scr = XDefaultScreen(dpy);
                Window root_window = XRootWindow(dpy, scr);
                XWarpPointer(dpy, Rc_None, root_window, 0, 0, 0, 0, x, y);
                XFlush(dpy);
#endif
                break;
            }
            case ServerOperator::MsLU:
            {
#ifdef CuUtil_Platform_Windows
                INPUT input{};
                ZeroMemory(&input, sizeof(INPUT));
                input.type = INPUT_MOUSE;
                input.mi.dwFlags = MOUSEEVENTF_LEFTUP;

                if (SendInput(1, &input, sizeof(INPUT)) != 1)
                {
                    LogErr("SendInput failed: {}", static_cast<uint64_t>(HRESULT_FROM_WIN32(GetLastError())));
                }
#endif
            }
            break;
            case ServerOperator::MsLD:
            {
#ifdef CuUtil_Platform_Windows
                INPUT input{};
                ZeroMemory(&input, sizeof(INPUT));
                input.type = INPUT_MOUSE;
                input.mi.dwFlags = MOUSEEVENTF_LEFTDOWN;

                if (SendInput(1, &input, sizeof(INPUT)) != 1)
                {
                    LogErr("SendInput failed: {}", static_cast<uint64_t>(HRESULT_FROM_WIN32(GetLastError())));
                }
#endif
            }
            break;
            case ServerOperator::MsRU:
            {
#ifdef CuUtil_Platform_Windows
                INPUT input{};
                ZeroMemory(&input, sizeof(INPUT));
                input.type = INPUT_MOUSE;
                input.mi.dwFlags = MOUSEEVENTF_RIGHTUP;

                if (SendInput(1, &input, sizeof(INPUT)) != 1)
                {
                    LogErr("SendInput failed: {}", static_cast<uint64_t>(HRESULT_FROM_WIN32(GetLastError())));
                }
#endif
            }
            break;
            case ServerOperator::MsRD:
            {
#ifdef CuUtil_Platform_Windows
                INPUT input{};
                ZeroMemory(&input, sizeof(INPUT));
                input.type = INPUT_MOUSE;
                input.mi.dwFlags = MOUSEEVENTF_RIGHTDOWN;

                if (SendInput(1, &input, sizeof(INPUT)) != 1)
                {
                    LogErr("SendInput failed: {}", static_cast<uint64_t>(HRESULT_FROM_WIN32(GetLastError())));
                }
#endif
            }
            break;
            case ServerOperator::KeDn:
            {
                const auto data = nlohmann::json::parse(co_await RcUtil::ReadString2(clientSocket));
                const auto key = static_cast<RcKey>(data["key"].get<std::underlying_type_t<RcKey>>());

#ifdef CuUtil_Platform_Windows
                if (const auto p = keyMap.find(key); p != keyMap.end())
                {
                    INPUT input{};
                    ZeroMemory(&input, sizeof(INPUT));
                    input.type = INPUT_KEYBOARD;
                    input.ki.wVk = p->second;

                    if (SendInput(1, &input, sizeof(INPUT)) != 1)
                    {
                        LogErr("SendInput failed: {}", static_cast<uint64_t>(HRESULT_FROM_WIN32(GetLastError())));
                    }
                }
                else
                {
                    LogWarn("ignore key: {}", CuEnum::ToString(key));
                }
#else
#endif
            }
            break;
            case ServerOperator::KeUp:
            {
                const auto data = nlohmann::json::parse(co_await RcUtil::ReadString2(clientSocket));
                const auto key = static_cast<RcKey>(data["key"].get<std::underlying_type_t<RcKey>>());

#ifdef CuUtil_Platform_Windows
                if (const auto p = keyMap.find(key); p != keyMap.end())
                {
                    INPUT input{};
                    ZeroMemory(&input, sizeof(INPUT));
                    input.type = INPUT_KEYBOARD;
                    input.ki.wVk = p->second;
                    input.ki.dwFlags = KEYEVENTF_KEYUP;

                    if (SendInput(1, &input, sizeof(INPUT)) != 1)
                    {
                        LogErr("SendInput failed: {}", static_cast<uint64_t>(HRESULT_FROM_WIN32(GetLastError())));
                    }
                }
                else
                {
                    LogWarn("ignore key: {}", CuEnum::ToString(key));
                }
#else
#endif
            }
            break;
            case ServerOperator::Auth:
                LogErr("unexpect operator Auth");
                break;
            default:
                throw MakeExcept("not impl");
            }
        }
    }

    // // nop
    // asio::awaitable<void> Writer()
    // {
    //
    // }

    asio::awaitable<ServerOperator> ReadOperator()
    {
        OperatorBuffer buffer;

        std::error_code ec;
        co_await async_read(clientSocket, asio::buffer(buffer), redirect_error(RcUtil_UseAwaitable, ec));
        if (ec)
            throw MakeApiExcept("asio::async_read", "{}", ec.message());
        LogDebug("read: {}", std::string_view(buffer.data(), buffer.size()));

        co_return CuEnum::TryFromString<ServerOperator>(std::string_view(buffer.data(), buffer.size()));
    }

    asio::awaitable<std::string> ReadPassword()
    {
        auto pw = co_await RcUtil::ReadString1(clientSocket);
        co_return pw;
    }
};

asio::awaitable<void> SrvListener(asio::ip::tcp::acceptor acceptor)
{
    LogDebug("start listener");
    try
    {
        VideoServer vidSrv{};

        while (true)
        {
            co_await std::make_shared<Client>(co_await acceptor.async_accept(RcUtil_UseAwaitable), vidSrv)->Start();
        }
    }
    catch (const std::exception &ex)
    {
        LogErr("{}", ex.what());
    }
}

int main(int argc, const char *argv[])
{
#pragma region Args
    CuArgs::Argument<std::string, 1, true> passwordArg{
        "-p",
        "password (length <= 255)",
        "123"};
    passwordArg.Validate = [](const auto &v)
    {
        if (v.length() > 255)
            throw MakeExcept("password too long");
        return true;
    };
    CuArgs::Argument urlPortArg{
        "--url-video-port",
        CuStr::Format("url video port. default: {}", LaunchConfig.UrlPort),
        LaunchConfig.UrlPort};
    CuArgs::Argument urlHostnameArg{
        "--url-video-hostname",
        CuStr::Format("url video hostname. default: {}", LaunchConfig.UrlHostname),
        LaunchConfig.UrlHostname};
    CuArgs::Argument hostnameArg{
        "--hostname",
        CuStr::Format("listen hostname. default: {}", LaunchConfig.Hostname),
        LaunchConfig.Hostname};
    CuArgs::Argument serverPortArg{
        "--server-port",
        CuStr::Format("server port. default: {}", LaunchConfig.ServerPort),
        LaunchConfig.ServerPort};
    CuArgs::Argument videoPortArg{
        "--video-port",
        CuStr::Format("video port. default: {}", LaunchConfig.VideoPort),
        LaunchConfig.VideoPort};
    CuArgs::EnumArgument logLevelArg{
        "--loglevel",
        CuStr::Format("log level {}. default: {}", CuEnum::Strings<CuLog::LogLevel>(), CuEnum::ToString(LaunchConfig.LogLevelValue)),
        LaunchConfig.LogLevelValue};

    CuArgs::Arguments args{};
    args.Add(passwordArg, urlPortArg, urlHostnameArg, hostnameArg, serverPortArg, videoPortArg, logLevelArg);

    try
    {
        args.Parse(argc, argv);
        CuConsole::WriteLine(args.GetValuesDesc());

        LaunchConfig.Password = args.Value(passwordArg);
        LaunchConfig.UrlPort = args.Value(urlPortArg);
        LaunchConfig.UrlHostname = args.Value(urlHostnameArg);
        LaunchConfig.ServerPort = args.Value(serverPortArg);
        LaunchConfig.VideoPort = args.Value(videoPortArg);
        LaunchConfig.LogLevelValue = args.Value(logLevelArg);

        Log.Level = LaunchConfig.LogLevelValue;
    }
    catch (const std::exception &ex)
    {
        CuConsole::Error::WriteLine(ex.what());
        CuConsole::Error::WriteLine("usage:");
        CuConsole::Error::WriteLine(args.GetDesc());
        return EXIT_FAILURE;
    }
#pragma endregion Args

    static std::thread LogThread([]
                                 {
	    const std::unordered_map<CuLog::LogLevel, CuConsole::Color> colorMap
        {
            { CuLog::LogLevel::None, CuConsole::Color::White },
            { CuLog::LogLevel::Error, CuConsole::Color::Red },
            { CuLog::LogLevel::Warn, CuConsole::Color::Yellow },
            { CuLog::LogLevel::Info, CuConsole::Color::White },
            { CuLog::LogLevel::Verb, CuConsole::Color::Gray },
            { CuLog::LogLevel::Debug, CuConsole::Color::Blue }
        };

        while (true)
        {
            const auto [level, raw] = Log.Chan.Read();
            const auto& [time, src, msg] = raw;

            auto out = CuStr::FormatU8("[{}] [{}] {}{}", CuEnum::ToString(level), LogMsg::LogTime(time), src, msg);
            if (out[out.length() - 1] == u8'\n') out.erase(out.length() - 1);

            SetForegroundColor(colorMap.at(level));
            CuConsole::WriteLine(CuStr::ToDirtyUtf8StringView(out));

            if (level == CuLog::LogLevel::None)
            {
                break;
            }
        } });

    try
    {
        asio::io_context srvCtx(1);
        co_spawn(srvCtx, SrvListener(asio::ip::tcp::acceptor{srvCtx, {asio::ip::tcp::v4(), LaunchConfig.ServerPort}}), asio::detached);

        asio::signal_set signals(srvCtx, SIGINT, SIGTERM);
        signals.async_wait([&](auto, auto)
                           {
            LogErr("signal: exit");
            srvCtx.stop(); });

        srvCtx.run();
    }
    catch (const std::exception &ex)
    {
        LogErr(ex.what());
    }

    LogNone("exit.");
    LogThread.join();
}
