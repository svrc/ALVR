#include "CEncoder.h"

CEncoder::CEncoder()
    : m_bExiting(false)
    , m_targetTimestampNs(0) {
    m_encodeFinished.Set();
}

CEncoder::~CEncoder() {
    if (m_videoEncoder) {
        m_videoEncoder->Shutdown();
        m_videoEncoder.reset();
    }
}

void CEncoder::Initialize(std::shared_ptr<CD3DRender> d3dRender) {
    for (int i = 0; i < 4; i++) {
        m_FrameRender[i] = std::make_shared<FrameRender>(d3dRender);
        m_FrameRender[i]->Startup();
    }
    uint32_t encoderWidth, encoderHeight;
    m_FrameRender[0]->GetEncodingResolution(&encoderWidth, &encoderHeight);

    Exception vceException;
    Exception nvencException;
#ifdef ALVR_GPL
    Exception swException;
    if (Settings::Instance().m_force_sw_encoding) {
        try {
            Debug("Try to use VideoEncoderSW.\n");
            m_videoEncoder
                = std::make_shared<VideoEncoderSW>(d3dRender, encoderWidth, encoderHeight);
            m_videoEncoder->Initialize();
            return;
        } catch (Exception e) {
            swException = e;
        }
    }
#endif

    try {
        Debug("Try to use VideoEncoderAMF.\n");
        m_videoEncoder = std::make_shared<VideoEncoderAMF>(d3dRender, encoderWidth, encoderHeight);
        m_videoEncoder->Initialize();
        return;
    } catch (Exception e) {
        vceException = e;
    }
    try {
        Debug("Try to use VideoEncoderNVENC.\n");
        m_videoEncoder
            = std::make_shared<VideoEncoderNVENC>(d3dRender, encoderWidth, encoderHeight);
        m_videoEncoder->Initialize();
        return;
    } catch (Exception e) {
        nvencException = e;
    }
#ifdef ALVR_GPL
    try {
        Debug("Try to use VideoEncoderSW.\n");
        m_videoEncoder = std::make_shared<VideoEncoderSW>(d3dRender, encoderWidth, encoderHeight);
        m_videoEncoder->Initialize();
        return;
    } catch (Exception e) {
        swException = e;
    }
    throw MakeException(
        "All VideoEncoder are not available. VCE: %s, NVENC: %s, SW: %s",
        vceException.what(),
        nvencException.what(),
        swException.what()
    );
#else
    throw MakeException(
        "All VideoEncoder are not available. VCE: %s, NVENC: %s",
        vceException.what(),
        nvencException.what()
    );
#endif
}

void CEncoder::SetViewsConfig(
    vr::HmdRect2_t projLeft,
    vr::HmdMatrix34_t eyeToHeadLeft,
    vr::HmdRect2_t projRight,
    vr::HmdMatrix34_t eyeToHeadRight
) {
    for (int i = 0; i < 4; i++) {
        m_FrameRender[i]->SetViewsConfig(projLeft, eyeToHeadLeft, projRight, eyeToHeadRight);
    }
}

bool CEncoder::CopyToStaging(
    ID3D11Texture2D* pTexture[][2],
    vr::VRTextureBounds_t bounds[][2],
    vr::HmdMatrix34_t poses[],
    int layerCount,
    bool recentering,
    uint64_t presentationTime,
    uint64_t targetTimestampNs,
    const std::string& message,
    const std::string& debugText
) {
    m_presentationTime = presentationTime;
    m_targetTimestampNs = targetTimestampNs;

    m_nextFrameIdx = (m_nextFrameIdx + 1) % 4;

    m_FrameRender[m_nextFrameIdx]->Startup();

    m_FrameRender[m_nextFrameIdx]->RenderFrame(
        pTexture, bounds, poses, layerCount, recentering, message, debugText
    );
    return true;
}

void CEncoder::QueueForEncoding() {
    if (m_FrameRender[m_nextFrameIdx]->GetTexture()) {
        m_videoEncoder->QueueForEncoding(
            m_FrameRender[m_nextFrameIdx]->GetTexture().Get(),
            m_presentationTime,
            m_targetTimestampNs,
            m_scheduler.CheckIDRInsertion()
        );
    }
}

void CEncoder::RunConsumer() {
    Debug("CEncoder: Start consumer thread. Id=%d\n", GetCurrentThreadId());
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_MOST_URGENT);

    while (!m_bExiting) {
        m_newFrameReady.Wait();

        // If m_newFrameReady triggered from ::Stop, exit the thread
        if (m_bExiting) {
            m_encodeFinished.Set();
            return;
        }

        QueueForEncoding();

        m_encodeFinished.Set();
    }
}

void CEncoder::RunProducer() {
    Debug("CEncoder: Start producer thread. Id=%d\n", GetCurrentThreadId());
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_MOST_URGENT);

    while (!m_bExiting) {
        m_videoEncoder->TransmitAvailable();
        if (m_bExiting)
            break;
    }
}

void CEncoder::Stop() {
    m_bExiting = true;
    m_newFrameReady.Set();
    Join();
    for (int i = 0; i < 4; i++) {
        m_FrameRender[i].reset();
    }
}

void CEncoder::NewFrameReady() {
    m_encodeFinished.Reset();
    m_newFrameReady.Set();
}

void CEncoder::WaitForEncode() { m_encodeFinished.Wait(); }

void CEncoder::OnStreamStart() { m_scheduler.OnStreamStart(); }

void CEncoder::OnPacketLoss() { m_scheduler.OnPacketLoss(); }

void CEncoder::InsertIDR() { m_scheduler.InsertIDR(); }

void CEncoder::CaptureFrame() { }
