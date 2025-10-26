import mujoco

# This automatically handles conversion internally
model = mujoco.MjModel.from_xml_path("assets/descriptions/bittle/urdf/bittle.urdf")

# If you want to save the converted model
xml_string = mujoco.mj_saveLastXML("assets/descriptions/bittle/mjcf/bittle.mjcf", model)