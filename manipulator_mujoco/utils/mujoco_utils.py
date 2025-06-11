import mujoco
import numpy as np


def get_site_jac(model, data, site_id):
    """Return the Jacobian' translational component of the end-effector of
    the corresponding site id.
    """
    if isinstance(site_id, str):
        site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, site_id)
    elif isinstance(site_id, int):
        site_id = site_id
    else:
        raise TypeError("site_id must be a string name or integer ID")
    # print("site_id", site_id)
    # print("model", model)
    # print("data", data)
    jacp = np.zeros((3, model.nv))
    jacr = np.zeros((3, model.nv))
    mujoco.mj_jacSite(model, data, jacp, jacr, site_id)
    jac = np.vstack([jacp, jacr])

    return jac

def get_fullM(model, data):
    M = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, M, data.qM)
    return M