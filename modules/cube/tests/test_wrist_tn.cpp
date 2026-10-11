#include <cmath>
#include <iostream>
#include <memory>
#include <stdexcept>

#include "rm/rx_wrist_tn.hpp"

int main() {
  try {
    const char* xml = R"(
<mujoco><option gravity="0 0 0" timestep=".001" integrator="implicitfast"/>
<worldbody>
<body name="A"><joint name="A_yaw" type="hinge"/><geom type="sphere" size=".01" mass="1"/></body>
<body name="B" pos="1 0 0"><joint name="B_yaw" type="hinge"/><geom type="sphere" size=".01" mass="1"/></body>
</worldbody><actuator>
<position name="A_yaw_drive" joint="A_yaw" kp="20" kv="1" forcelimited="true" forcerange="-30 30"/>
<position name="B_yaw_drive" joint="B_yaw" kp="20" kv="1" forcelimited="true" forcerange="-30 30"/>
</actuator></mujoco>)";
    mjVFS vfs;
    mj_defaultVFS(&vfs);
    mj_addBufferVFS(&vfs, "test.xml", xml, std::char_traits<char>::length(xml));
    char error[1024]{};
    std::unique_ptr<mjModel, decltype(&mj_deleteModel)> m(
        mj_loadXML("test.xml", &vfs, error, sizeof(error)), mj_deleteModel);
    mj_deleteVFS(&vfs);
    if (!m) throw std::runtime_error(error);
    std::unique_ptr<mjData, decltype(&mj_deleteData)> d(mj_makeData(m.get()), mj_deleteData);
    rm::cube::RxWristTn tn(m.get());
    d->ctrl[0] = 100;
    d->ctrl[1] = -100;
    tn.before_step(m.get(), d.get());
    const double limit = tn.axes[0].step_limit;
    mj_step(m.get(), d.get());
    tn.after_step(d.get());
    if (std::abs(d->actuator_force[0] - limit) > 1e-10 ||
        std::abs(d->actuator_force[1] + limit) > 1e-10)
      throw std::runtime_error("Positive/negative motor effort not bounded by TN");
    mj_resetData(m.get(), d.get());
    d->qvel[0] = 100;
    d->qvel[1] = -100;
    d->ctrl[0] = 100;
    d->ctrl[1] = -100;
    tn.before_step(m.get(), d.get());
    mj_step(m.get(), d.get());
    tn.after_step(d.get());
    if (d->actuator_force[0] != 0 || d->actuator_force[1] != 0 ||
        std::abs(d->qvel[0] - 100) > 1e-10 || std::abs(d->qvel[1] + 100) > 1e-10)
      throw std::runtime_error("Overspeed must suppress motor output without clipping velocity");
    std::cout << "TN双向力矩和超速惯性滑行检查通过\n";
    return 0;
  } catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
  }
}
