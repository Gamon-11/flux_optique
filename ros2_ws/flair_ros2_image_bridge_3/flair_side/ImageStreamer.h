// filename:   ImageStreamer.h
//
// purpose:    minimal TCP image streamer for FLAIR -> ROS2 (see ImageStreamer.cpp
//             for the wire protocol). Attach it directly to a Camera device
//             (e.g. uav->GetVerticalCamera()) and every new frame produced by
//             that camera is forwarded over TCP.

#ifndef IMAGESTREAMER_H
#define IMAGESTREAMER_H

#include <IODevice.h>
#include <string>

namespace flair {
namespace core {
class TcpSocket;
class io_data;
}
}

namespace flair {
namespace filter {

class ImageStreamer : public core::IODevice {
public:
    // parent: the Camera device to stream (e.g. uav->GetVerticalCamera())
    // ipAddress/ipPort: where the ROS2 receiver node is listening
    ImageStreamer(const core::IODevice *parent, std::string name,
                  std::string ipAddress, int ipPort);
    ~ImageStreamer();

private:
    void UpdateFrom(const core::io_data *data) override;
    bool SendAll(const char *data, size_t len);

    core::TcpSocket *sendingSocket;
    bool headerSent;
};

} // end namespace filter
} // end namespace flair

#endif // IMAGESTREAMER_H
