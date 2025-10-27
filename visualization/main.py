import mujoco
import mujoco.viewer

import os
from init import logger
import constants

'''
Converts URDF model to MJCF, which is Mujoco compatible
'''
def convertToMJCF(URDF_PATH, MJCF_PATH):
    # Loads URDF model into memory and saves as MJCF
    model = mujoco.MjModel.from_xml_path(URDF_PATH)
    mujoco.mj_saveLastXML(MJCF_PATH, model)

    logger.info(f"Converted URDF to MJCF and saved to {MJCF_PATH}")
    logger.info(f"Model stats: {model.nbody} bodies, {model.njnt} joints, "
                f"{model.nv} DOF, {model.nu} actuators, {model.nmesh} meshes")

if __name__ == "__main__":
    # If MJCF file does not exist, convert from URDF
    if not os.path.exists(constants.BITTLE_MJCF_PATH):
        logger.info("MJCF file not found, converting from URDF...")
        convertToMJCF(constants.BITTLE_URDF_PATH, constants.BITTLE_MJCF_PATH)
    else:
        logger.info("MJCF file exists, proceeding to visualization")

    logger.info("Loading Mujoco model")

    # Load environment model into memory
    model = mujoco.MjModel.from_xml_path(constants.BITTLE_ENVIRONMENT_PATH)
    data = mujoco.MjData(model)
    
    # Initialize with default XML values
    mujoco.mj_resetData(model, data)
    mujoco.mj_forward(model, data)
    
    logger.info(f"Model stats: {model.nbody} bodies, {model.njnt} joints, "
                f"{model.nv} DOF, {model.nu} actuators, {model.nmesh} meshes")
    
    # Launch viewer
    logger.info("Launching Mujoco viewer")
    mujoco.viewer.launch(model, data)

    