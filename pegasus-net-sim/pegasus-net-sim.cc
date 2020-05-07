/* Raunak Mukhia 2020 */
#include "Pegasus.h"

std::vector<std::string> drones = {
  "iris_0",
  "iris_1",
  "iris_2"
};




int main(int argc, char **argv)
{
  Pegasus *pegasus = Pegasus::GetInstance();
  pegasus->Run(argc, argv, drones);

}
