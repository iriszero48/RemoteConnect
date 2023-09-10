#pragma once

#include <cstdint>
#include <mutex>

#include <QOpenGLWidget>
#include <QOpenGLFunctions_4_5_Core>
#include <QOpenGLShaderProgram>
#include <QOpenGLBuffer>
#include <QOpenGLVertexArrayObject>
#include <QPainter>
#include <QMouseEvent>
#include <QTcpSocket>

#include <nlohmann/json.hpp>

#include <Image/Image.hpp>
#include <Image/File.hpp>
#include <Convert/Convert.hpp>
#include <Bit/Bit.hpp>

#include "RcUtility.hpp"

class GLWidget : public QOpenGLWidget, protected QOpenGLFunctions_4_5_Core
{
    Q_OBJECT
private:
    QOpenGLBuffer vbo;
    QOpenGLVertexArrayObject vao;
    QOpenGLShaderProgram *program{};

    int currentHeight = 0;
    int currentWidth = 0;

    int imageHeight = 0;
    int imageWidth = 0;
    GLuint tex{};

    int fps = 0;
    int frameCount = 0;
    std::chrono::high_resolution_clock::time_point lastDrawTime{};

    using VboType = GLfloat;

#define MakeBufferItemDesc(prefix, location, offset, offsetInBytes, sizeInPoints, tupleSize, type) \
    static constexpr int prefix##Location = location;                                              \
    static constexpr size_t prefix##Offset = offset;                                               \
    static constexpr size_t prefix##OffsetInBytes = offsetInBytes;                                 \
    static constexpr size_t prefix##SizeInPoints = sizeInPoints;                                   \
    static constexpr size_t prefix##TupleSize = tupleSize;                                         \
    static constexpr size_t prefix##Size = prefix##TupleSize * prefix##SizeInPoints;               \
    static constexpr size_t prefix##SizeInBytes = BorderSize * sizeof(type)
#define MakeBufferItemDescBegin(prefix, location, sizeInPoints, tupleSize, type) \
    MakeBufferItemDesc(prefix, location, 0, 0, sizeInPoints, tupleSize, type)
#define MakeBufferItemDescNext(prefix, location, prev, sizeInPoints, tupleSize, type) \
    MakeBufferItemDesc(prefix, location, prev##Size, prev##SizeInBytes, sizeInPoints, tupleSize, type)

    MakeBufferItemDescBegin(Border, 0, 4, 2, VboType);
    MakeBufferItemDescNext(TexCoord, 1, Border, 4, 2, VboType);

#undef MakeBufferItemDesc
#undef MakeBufferItemDescBegin
#undef MakeBufferItemDescNext

    static constexpr size_t VboSize = BorderSize + TexCoordSize;
    static constexpr size_t VboSizeInBytes = BorderSizeInBytes + TexCoordSizeInBytes;
    std::array<GLfloat, VboSize> quadData{0};

    std::shared_ptr<QTcpSocket> sock{};

    std::optional<std::array<GLfloat, BorderSize>> border{};

public:
    GLWidget(std::shared_ptr<QTcpSocket> sock, QWidget *parent = nullptr) : QOpenGLWidget(parent), sock(std::move(sock))
    {
        setAutoFillBackground(false);
        setMouseTracking(true);

        ResetBorder();
        ResetTexCoord();
    }

    void SetCurrentFrame(const CuImg::ImageRGBA &img)
    {
        const auto h = img.Height();
        const auto w = img.Width();

        if (imageHeight != h || imageWidth != w)
        {
            imageHeight = h;
            imageWidth = w;

            makeCurrent();
            AdjQuad();
            doneCurrent();
        }

        glBindTexture(GL_TEXTURE_2D, tex);

        glTexStorage2D(GL_TEXTURE_2D, 1, GL_RGBA8, w, h);
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w, h, GL_RGBA, GL_UNSIGNED_BYTE, img.Data());

        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);

        update();
    }

private:
    bool IsPlaying() const
    {
        return sock && imageHeight && imageWidth;
    }

protected:
    void closeEvent(QCloseEvent *event) override
    {
        if (IsPlaying())
        {
            WriteOperator(*sock, ServerOperator::Exit);
        }
    }

    void keyPressEvent(QKeyEvent *event) override
    {
        if (!IsPlaying())
            return;
        WriteOperator(*sock, ServerOperator::KeDn);
        WriteString2(*sock, nlohmann::json{{"key", event->key()}}.dump());
    }

    void keyReleaseEvent(QKeyEvent *event) override
    {
        if (!IsPlaying())
            return;
        WriteOperator(*sock, ServerOperator::KeUp);
        WriteString2(*sock, nlohmann::json{{"key", event->key()}}.dump());
    }

    void mousePressEvent(QMouseEvent *event) override
    {
        if (!IsPlaying())
            return;
        static std::unordered_map<Qt::MouseButton, ServerOperator> evMap{
            {Qt::LeftButton, ServerOperator::MsLD},
            {Qt::RightButton, ServerOperator::MsRD}};

        if (const auto p = evMap.find(event->button()); p != evMap.end())
        {
            WriteOperator(*sock, p->second);
        }
    }

    void mouseReleaseEvent(QMouseEvent *event) override
    {
        if (!IsPlaying())
            return;
        static std::unordered_map<Qt::MouseButton, ServerOperator> evMap{
            {Qt::LeftButton, ServerOperator::MsLU},
            {Qt::RightButton, ServerOperator::MsRU}};

        if (const auto p = evMap.find(event->button()); p != evMap.end())
        {
            WriteOperator(*sock, p->second);
        }
    }

    void mouseMoveEvent(QMouseEvent *event) override
    {
        if (!IsPlaying())
            return;
        static auto lastTime = std::chrono::system_clock::time_point{};

        const auto pos = event->position();
        const auto &bd = border;
        if (!bd)
            return;

        using Point = std::tuple<GLfloat, GLfloat>;
        const auto getPoint = [&](const size_t i)
        { return Point(bd->operator[](i * 2), bd->operator[](i * 2 + 1)); };
        const auto getX = [&](const Point &p)
        { return std::get<0>(p); };
        const auto getY = [&](const Point &p)
        { return std::get<1>(p); };
        const auto p0 = getPoint(0);
        const auto p2 = getPoint(2);

        const auto halfW = static_cast<float>(currentWidth) / 2.f;
        const auto halfH = static_cast<float>(currentHeight) / 2.f;
        const auto trans = [&](const Point &p)
        { return Point(halfW * getX(p) + halfW, halfH * getY(p) + halfH); };
        const auto lt = trans(p0);
        const auto rb = trans(p2);

        const auto px = static_cast<float>(pos.x());
        const auto py = static_cast<float>(pos.y());

        const auto [ltx, lty] = lt;
        const auto [rbx, rby] = rb;

        // rbx <= px <= ltx, lty <= py <= rby
        if (px <= ltx && px >= rbx && py >= lty && py <= rby)
        {
            using namespace std::chrono_literals;

            if (const auto t = std::chrono::system_clock::now(); t - lastTime >= 50ms)
            {
                lastTime = t;

                const auto data = nlohmann::json{
                    {"x", (px - rbx) * imageWidth / (ltx - rbx)},
                    {"y", (py - lty) * imageHeight / (rby - lty)}}
                                      .dump();

                WriteOperator(*sock, ServerOperator::MsMv);
                WriteString2(*sock, data);
            }
        }
    }

public:
    static void WriteOperator(QTcpSocket &sock, const ServerOperator value)
    {
        sock.write(CuEnum::ToString(value).data(), 4);
    }

    template <typename T>
    static void WriteInt(QTcpSocket &sock, T value)
    {
        if constexpr (std::endian::native == std::endian::big)
        {
            value = CuBit::ByteSwap(value);
        }
        sock.write(reinterpret_cast<const char *>(&value), sizeof(T));
    }

    template <typename T>
    static void WriteString(QTcpSocket &sock, const std::string_view &str)
    {
        CuUtil_Assert(str.length() <= std::numeric_limits<T>::max(), Exception);

        const auto len = static_cast<T>(str.length());
        WriteInt(sock, len);
        sock.write(str.data(), len);
    }

    static inline auto WriteString2 = &WriteString<uint16_t>;

protected:
    void initializeGL() override
    {
        initializeOpenGLFunctions();

        program = new QOpenGLShaderProgram(this);
        program->addShaderFromSourceCode(QOpenGLShader::Vertex, R"(
        #version 330 core
        layout (location = 0) in vec2 in_position;
        layout (location = 1) in vec2 in_tex_coord;

        out vec2 tex_coord;

        void main(void)
        {
            gl_Position = vec4(in_position, 0.5, 1.0);
            tex_coord = in_tex_coord;
        })");
        program->addShaderFromSourceCode(QOpenGLShader::Fragment, R"(
        #version 330 core

        in vec2 tex_coord;

        layout (location = 0) out vec4 color;

        uniform sampler2D tex;

        void main(void)
        {
            color = texture(tex, tex_coord);
        })");

        program->link();

        program->bind();

        glGenTextures(1, &tex);

        {
            QOpenGLVertexArrayObject::Binder vb(&vao);
            vbo.create();
            vbo.bind();

            vbo.setUsagePattern(QOpenGLBuffer::UsagePattern::StaticDraw);
            vbo.allocate(quadData.data(), VboSizeInBytes);

            program->setAttributeBuffer(BorderLocation, GL_FLOAT, BorderOffsetInBytes, BorderTupleSize, 0);
            program->setAttributeBuffer(TexCoordLocation, GL_FLOAT, TexCoordOffsetInBytes, TexCoordTupleSize, 0);
            program->enableAttributeArray(BorderLocation);
            program->enableAttributeArray(TexCoordLocation);
        }
    }

    void paintGL() override
    {
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClearDepth(1.0);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glDisable(GL_CULL_FACE);

        QPainter qp(this);
        qp.beginNativePainting();
        {
            QOpenGLVertexArrayObject::Binder vb(&vao);

            program->bind();
            glBindTexture(GL_TEXTURE_2D, tex);

            glDrawArrays(GL_TRIANGLE_FAN, 0, BorderSizeInPoints);
        }
        qp.endNativePainting();
        qp.setOpacity(0.8);
        qp.setPen(QColorConstants::Gray);
        qp.drawText(QRect(0, 0, 100, 20), CuConv::ToString(fps)->c_str());
        qp.setOpacity(1.0);

        ++frameCount;

        const auto t = std::chrono::high_resolution_clock::now();
        const std::chrono::duration<float, std::milli> tc = t - lastDrawTime;
        constexpr auto tp = 2.f;
        if (const auto s = tc.count() / 1000.f; s >= tp)
        {
            fps = static_cast<int>(static_cast<float>(frameCount) / tp);
            frameCount = 0;
            lastDrawTime = t;
        }
    }

    void resizeGL(const int width, const int height) override
    {
        currentHeight = height;
        currentWidth = width;

        glViewport(0, 0, width, height);

        AdjQuad();
    }

private:
    template <size_t ItemOffset, size_t PointOffset, typename Arr, typename T = typename Arr::value_type>
    static constexpr void WritePoint2(Arr &arr, T &&x, T &&y)
    {
        arr[ItemOffset + PointOffset * 2 + 0] = x;
        arr[ItemOffset + PointOffset * 2 + 1] = y;
    }

    void ResetBorder()
    {
        static_assert(BorderTupleSize == 2);

        WritePoint2<BorderOffset, 0>(quadData, +1.0f, -1.0f);
        WritePoint2<BorderOffset, 1>(quadData, -1.0f, -1.0f);
        WritePoint2<BorderOffset, 2>(quadData, -1.0f, +1.0f);
        WritePoint2<BorderOffset, 3>(quadData, +1.0f, +1.0f);
    }

    void ResetTexCoord()
    {
        static_assert(TexCoordTupleSize == 2);

        WritePoint2<TexCoordOffset, 0>(quadData, 1.0f, 1.0f);
        WritePoint2<TexCoordOffset, 1>(quadData, 0.0f, 1.0f);
        WritePoint2<TexCoordOffset, 2>(quadData, 0.0f, 0.0f);
        WritePoint2<TexCoordOffset, 3>(quadData, 1.0f, 0.0f);
    }

    void AdjQuad()
    {
        if (!(imageHeight && imageWidth && currentHeight && currentWidth))
            return;

        auto offset = 0;
        GLfloat value;

        ResetBorder();

        if (const auto h = static_cast<float>(imageHeight * currentWidth) / imageWidth; h > currentHeight)
        {
            const auto w = static_cast<float>(imageWidth * currentHeight) / imageHeight;
            value = w / currentWidth;
        }
        else
        {
            value = h / currentHeight;
            offset = 1;
        }

        for (size_t i = 0; i < 4; ++i)
        {
            quadData[i * 2 + offset] *= value;
        }

        std::array<GLfloat, BorderSize> b{};
        std::copy_n(quadData.begin(), BorderSize, b.begin());
        border = b;

        {
            QOpenGLVertexArrayObject::Binder vb(&vao);
            vbo.bind();
            vbo.write(BorderOffsetInBytes, quadData.data(), BorderSizeInBytes);
        }
    }
};
