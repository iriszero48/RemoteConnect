#pragma once

#include <QMainWindow>
#include <QMenuBar>
#include <QByteArray>
#include <QBuffer>
#include <QTimer>
#include <QInputDialog>
#include <QMessageBox>
#include <QTcpSocket>

#include <nlohmann/json.hpp>

#include <filesystem>

#include <Video/Video.hpp>

#include "OGLWidget.hpp"
#include "RcUtility.hpp"

#include "Assert/Assert.hpp"

class MainWindow final : public QMainWindow
{
    Q_OBJECT

public:
    using VidType = CuVid::Decoder<>;
    using FrameType = VidType::VideoFrameType;
    using ImageType = CuImg::ImageRGBA;

    explicit MainWindow(QWidget *parent = nullptr)
    {
        auto *fileMenu = new QMenu("File");
        menuBar()->addMenu(fileMenu);

        auto *connAct = new QAction("Connect...");
        fileMenu->addAction(connAct);

        connect(this, &MainWindow::StartPlayEvent, this, &MainWindow::OnStartPlay);

        connect(connAct, &QAction::triggered, this, [&]()
                {
	        bool testOk = false;
            if (const auto retUrl = QInputDialog::getText(this, "Connect...", "URL:", QLineEdit::Normal, {}, &testOk); testOk)
            {
                if (retUrl.isEmpty())
                {
                    QMessageBox::warning(this, "Warning", "path empty!");
                    return;
                }

                const auto urlSplitPos = retUrl.lastIndexOf(':');
                if (urlSplitPos == -1)
                {
                    QMessageBox::warning(this, "Error", "error url format");
                    return;
                }

                const auto baseUrl = retUrl.sliced(0, urlSplitPos);
                bool portOk;
                const auto port = retUrl.sliced(urlSplitPos + 1).toUShort(&portOk);
                if (!portOk)
                {
                    QMessageBox::warning(this, "Error", "error url format");
                    return;
                }

                srvSock->connectToHost(baseUrl, port, QIODeviceBase::ReadWrite, QAbstractSocket::IPv4Protocol);
                if (!srvSock->waitForConnected())
                {
                    QMessageBox::warning(this, "Error", QString::fromStdU32String(CuStr::FormatU32("Cannot connect to {}:{}", baseUrl.toStdU32String(), port)));
                    return;
                }

                const auto retPw = QInputDialog::getText(this, "Connect...", "Password:", QLineEdit::Password, {}, &testOk);
                if (!testOk || retPw.isEmpty())
                {
                    QMessageBox::warning(this, "Error", "please type password");
                    return;
                }
                if (retPw.length() > 255)
                {
                    QMessageBox::warning(this, "Error", "password too long");
                    return;
                }

            	std::array<uint8_t, 2> lenStr{0, 0};
                lenStr[0] = static_cast<uint8_t>(retPw.length());
                const auto auth = CuStr::Appends(CuEnum::ToString(ServerOperator::Auth), reinterpret_cast<const char*>(lenStr.data()), retPw.toStdString());
                qint64 pass = srvSock->write(auth.data(), auth.length());
                CuAssert(pass == auth.length());
                if (!srvSock->waitForBytesWritten())
                {
                    QMessageBox::warning(this, "Error", "write Auth failed");
                    return;
                }

                std::string vidUrl{};
                std::array<uint8_t, sizeof(uint16_t)> vidUrlLenBuf{};
                auto* vidUrlLen = reinterpret_cast<uint16_t*>(vidUrlLenBuf.data());
                if (!srvSock->waitForReadyRead())
                {
                    QMessageBox::warning(this, "Error", "read address failed");
                    return;
                }
                pass = srvSock->read(reinterpret_cast<char*>(vidUrlLenBuf.data()), vidUrlLenBuf.size());
                CuAssert(pass == vidUrlLenBuf.size());
                if constexpr (std::endian::native == std::endian::big)
                {
                    *vidUrlLen = CuBit::ByteSwap(*vidUrlLen);
                }
                vidUrl.resize(*vidUrlLen);
                if (!srvSock->waitForReadyRead())
                {
                    QMessageBox::warning(this, "Error", "read address failed");
                    return;
                }
                pass = srvSock->read(vidUrl.data(), vidUrl.length());
                CuAssert(pass == vidUrl.length());

                const auto res = nlohmann::json::parse(vidUrl);
                if (!res["status"].get<bool>())
                {
                    QMessageBox::warning(this, "Error", QString::fromStdU32String(CuStr::ToU32String(CuStr::FromDirtyUtf8String(res["error"].get<std::string>()))));
                    return;
                }

	        	emit StartPlayEvent(CuStr::FromDirtyUtf8String(res["result"].get<std::string>()));
	        } });

        srvSock = std::make_shared<QTcpSocket>();
        oglWidget = new GLWidget(srvSock, this);
        setCentralWidget(oglWidget);
        connect(this, &MainWindow::RecvFrameEvent, this, &MainWindow::OnRecvFrame);
        connect(this, &MainWindow::RecvAudioEvent, this, &MainWindow::OnRecvAudio);

        vid.Config.DecodeType = CuVid::StreamTypeVideo;
        vid.Config.VideoHandler = [&](FrameType &frame)
        {
            auto img = std::make_shared<CuImg::ImageRGBA>();
            CuImg::Convert(frame, *img);
            emit RecvFrameEvent(std::move(img));
        };
    }

    ~MainWindow()
    {
        vidBg.request_stop();
    }

private:
    std::shared_ptr<QTcpSocket> srvSock = {};

    GLWidget *oglWidget;
    VidType vid;

    std::jthread vidBg;

    // QAudioOutput audioOutput;

signals:
    void RecvFrameEvent(std::shared_ptr<ImageType> frame);
    void RecvAudioEvent(QByteArray data);
    void StartPlayEvent(const std::filesystem::path &path);

public slots:
    void OnRecvFrame(const std::shared_ptr<ImageType> &frame) const
    {
        oglWidget->SetCurrentFrame(*frame);
    }

    static void OnRecvAudio(const QByteArray &) {}

    void OnStartPlay(const std::filesystem::path &path)
    {
        vid.Config.Input = path;
        vid.Config.Options
            .Set(u8"fflags", u8"nobuffer")
            .Set(u8"flags", u8"low_delay")
            .Set(u8"probesize", 32);
        try
        {
            vid.LoadFile();
            vid.FindStream();
        }
        catch (const std::exception &ex)
        {
            QMessageBox::warning(this, "Error", ex.what());
            return;
        }
        vidBg = std::jthread([&](const std::stop_token &token)
                             {
                                while (!vid.Eof() && !token.stop_requested()) vid.Read(); });
    }
};
