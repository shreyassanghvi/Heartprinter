#include "../../include/custom/Base.h"
#include <cmath>

Base::Base(const DOUBLE_POSITION_ANGLES_RECORD &anchor_, int id) : anchor(anchor_), cableLength(0.0), motor(id) {
}
double Base::computeCableDelta(const DOUBLE_POSITION_ANGLES_RECORD &src,
                               const DOUBLE_POSITION_ANGLES_RECORD &dest) const {
    double currentLength = computeCableLength(src);
    double desiredLength = computeCableLength(dest);
    return desiredLength - currentLength;
}

double Base::computeCableLength(const DOUBLE_POSITION_ANGLES_RECORD &platform) const {
    double dx = anchor.x - platform.x;
    double dy = anchor.y - platform.y;
    double dz = anchor.z - platform.z;
    spdlog::info("dx: {}, dy: {}, dz: {}", dx, dy, dz);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void Base::updateCableLength(const DOUBLE_POSITION_ANGLES_RECORD &platform) {
    auto prevCableLength = cableLength;
    cableLength = computeCableLength(platform);
    spdlog::info("prevCableLength: {}, cableLength: {}", prevCableLength, cableLength);
}

double Base::getCableLength() const {
    return cableLength;
}

DOUBLE_POSITION_ANGLES_RECORD Base::getAnchor() const {
    return anchor;
}

Motor &Base::getMotor() {
    return motor;
}
