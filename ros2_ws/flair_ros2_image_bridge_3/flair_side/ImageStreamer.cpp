// filename:   ImageStreamer.cpp
//
// purpose:    minimal TCP image streamer for FLAIR -> ROS2
//
// Wire protocol (must match the ROS2-side receiver exactly):
//   - once, right after the TCP connection is established:
//       8 bytes = width (uint32, native/little-endian) + height (uint32, native/little-endian)
//   - then, for every camera frame:
//       width * height * 3 raw bytes, BGR, row-major, no padding
//     (this is exactly flair::core::Image::buffer for a Camera created with
//     Image::Type::Format::BGR, which is what FLAIR's simulated cameras use)
//
// Note: the connection is a *blocking* connect (see TcpSocket::Connect's
// default TIME_INFINITE timeout) -> the ROS2 receiver must already be
// listening before this demo is started, otherwise the constructor blocks
// until it is.

#include "ImageStreamer.h"
#include <TcpSocket.h>
#include <FrameworkManager.h>
#include <Image.h>
#include <stdint.h>

using namespace flair::core;

namespace flair {
namespace filter {

ImageStreamer::ImageStreamer(const core::IODevice *parent, std::string name,
                              std::string ipAddress, int ipPort)
    : IODevice(parent, name), headerSent(false) {
    // blockOnSend=true, blockOnReceive=true (we never receive on this socket)
    sendingSocket = new TcpSocket(getFrameworkManager(), name + "_socket", true, true);

    if (!sendingSocket->Connect(ipPort, ipAddress)) {
        // NB: Warn (not Err) on purpose. Object::Error() sets a framework-wide
        // error flag (FrameworkManager::ErrorOccured()), which every
        // *Controller::Run() checks before starting its loop (see e.g.
        // HostEthController::Run()) and refuses to run on. A failed image
        // streaming connection is not a flight-safety issue and must not
        // block the joystick/controller loop or trigger a full SafeStop of
        // the whole demo.
        Warn("ImageStreamer: connection to %s:%d failed\n", ipAddress.c_str(), ipPort);
    } else {
        Printf("ImageStreamer: connected to %s:%d\n", ipAddress.c_str(), ipPort);
    }
}

ImageStreamer::~ImageStreamer() {
    // sendingSocket is a child Object of the FrameworkManager (like every
    // other FLAIR device in this demo) and is torn down with it; it is not
    // deleted explicitly here, matching the rest of this codebase (see e.g.
    // CircleFollower::~CircleFollower()).
}

void ImageStreamer::UpdateFrom(const io_data *data) {
    Image *img = (Image *)data;

    data->GetMutex();

    if (!headerSent) {
        uint32_t header[2] = {(uint32_t)img->GetDataType().GetWidth(),
                               (uint32_t)img->GetDataType().GetHeight()};
        if (!SendAll((char *)header, sizeof(header))) {
            Warn("ImageStreamer: header send failed\n");
        }
        headerSent = true;
    }

    ssize_t imageSize = img->GetDataType().GetSize();
    if (!SendAll(img->buffer, imageSize)) {
        Warn("ImageStreamer: send failed\n");
    }

    data->ReleaseMutex();

    ProcessUpdate(img);
}

// TcpSocket::SendMessage() wraps a single send() call, which on a blocking
// TCP socket is allowed to write fewer bytes than requested (common on large
// buffers over loopback). Without looping here, frames desync from the
// receiver's framing over time. GetSize()/(sizeof header) are always small
// enough that len fits in a ssize_t.
bool ImageStreamer::SendAll(const char *data, size_t len) {
    size_t sent = 0;
    while (sent < len) {
        ssize_t n = sendingSocket->SendMessage(data + sent, len - sent, 0);
        if (n <= 0) {
            return false;
        }
        sent += (size_t)n;
    }
    return true;
}

} // end namespace filter
} // end namespace flair
