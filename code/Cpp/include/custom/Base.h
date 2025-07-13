#ifndef BASE_H
#define BASE_H
#include "../Trackstar/ATC3DG.h"
#include "Motor.h"

class Base {
public:
    explicit Base(const DOUBLE_POSITION_ANGLES_RECORD &anchor_);

    double computeCableLength(const DOUBLE_POSITION_ANGLES_RECORD &platform) const;

    void updateCableLength(const DOUBLE_POSITION_ANGLES_RECORD &platform);

    double computeCableDelta(const DOUBLE_POSITION_ANGLES_RECORD &src,
                             const DOUBLE_POSITION_ANGLES_RECORD &dest) const;

    double getCableLength() const;

    DOUBLE_POSITION_ANGLES_RECORD getAnchor() const;

    Motor &getMotor(); // Accessor for the motor

private:
    DOUBLE_POSITION_ANGLES_RECORD anchor;
    double cableLength;
    Motor motor;
};

#endif // BASE_H
