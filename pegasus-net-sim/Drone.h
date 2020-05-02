//
// Created by rmukhia on 2/5/63.
//

#ifndef PEGASUS_SIM_DRONE_H
#define PEGASUS_SIM_DRONE_H

class Drone {
  public:
    Drone(): posX{0}, posY{0}, posZ{0} {}
    Drone(int x, int y, int z): posX(x), posY(y), posZ(z) {}
    int posX;
    int posY;
    int posZ;
};

#endif //PEGASUS_SIM_DRONE_H
