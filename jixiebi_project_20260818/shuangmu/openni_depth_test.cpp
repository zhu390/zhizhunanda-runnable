#include <OpenNI.h>
#include <iostream>
#include <limits>

int main() {
    openni::Status rc = openni::OpenNI::initialize();
    if (rc != openni::STATUS_OK) {
        std::cerr << "OpenNI initialize failed: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        return 1;
    }

    openni::Array<openni::DeviceInfo> devices;
    openni::OpenNI::enumerateDevices(&devices);
    std::cout << "device count: " << devices.getSize() << std::endl;

    for (int i = 0; i < devices.getSize(); ++i) {
        const openni::DeviceInfo& info = devices[i];
        std::cout << "device " << i
                  << " name=" << info.getName()
                  << " vendor=" << info.getVendor()
                  << " uri=" << info.getUri()
                  << std::endl;
    }

    openni::Device device;
    rc = device.open(openni::ANY_DEVICE);
    if (rc != openni::STATUS_OK) {
        std::cerr << "device open failed: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        openni::OpenNI::shutdown();
        return 2;
    }

    if (!device.hasSensor(openni::SENSOR_DEPTH)) {
        std::cerr << "no depth sensor" << std::endl;
        device.close();
        openni::OpenNI::shutdown();
        return 3;
    }

    openni::VideoStream depth;
    rc = depth.create(device, openni::SENSOR_DEPTH);
    if (rc != openni::STATUS_OK) {
        std::cerr << "depth stream create failed: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        device.close();
        openni::OpenNI::shutdown();
        return 4;
    }

    rc = depth.start();
    if (rc != openni::STATUS_OK) {
        std::cerr << "depth stream start failed: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        depth.destroy();
        device.close();
        openni::OpenNI::shutdown();
        return 5;
    }

    std::cout << "depth stream started" << std::endl;

    for (int frame_id = 0; frame_id < 30; ++frame_id) {
        openni::VideoFrameRef frame;
        rc = depth.readFrame(&frame);
        if (rc != openni::STATUS_OK || !frame.isValid()) {
            std::cerr << "read frame failed: "
                      << openni::OpenNI::getExtendedError() << std::endl;
            continue;
        }

        int width = frame.getWidth();
        int height = frame.getHeight();
        const openni::DepthPixel* data =
            static_cast<const openni::DepthPixel*>(frame.getData());

        int cx = width / 2;
        int cy = height / 2;
        int center = data[cy * width + cx];

        int valid_count = 0;
        int min_depth = std::numeric_limits<int>::max();
        int max_depth = 0;

        for (int i = 0; i < width * height; ++i) {
            int z = data[i];
            if (z > 0) {
                valid_count++;
                if (z < min_depth) min_depth = z;
                if (z > max_depth) max_depth = z;
            }
        }

        std::cout << "frame " << frame_id
                  << " size=" << width << "x" << height
                  << " center_mm=" << center
                  << " valid=" << valid_count
                  << " min_mm=" << (valid_count ? min_depth : 0)
                  << " max_mm=" << max_depth
                  << std::endl;
    }

    depth.stop();
    depth.destroy();
    device.close();
    openni::OpenNI::shutdown();
    return 0;
}
