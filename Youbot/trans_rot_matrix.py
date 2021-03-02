def trans_rot_matrix(rotation, translation):

    # translational and rotational matrices
    # first translate the point, then rotate in X, then rotate in Y and finally in Z

  import numpy as np

  xC = np.cos(rotation[0])
  xS = np.sin(rotation[0])
  yC = np.cos(rotation[1])
  yS = np.sin(rotation[1])
  zC = np.cos(rotation[2])
  zS = np.sin(rotation[2])
  dX = translation[0]
  dY = translation[1]
  dZ = translation[2]
  Translate_matrix = np.array([[1, 0, 0, dX],
                               [0, 1, 0, dY],
                               [0, 0, 1, dZ],
                               [0, 0, 0, 1]])
  Rotate_X_matrix = np.array([[1, 0, 0, 0],
                              [0, xC, -xS, 0],
                              [0, xS, xC, 0],
                              [0, 0, 0, 1]])
  Rotate_Y_matrix = np.array([[yC, 0, yS, 0],
                              [0, 1, 0, 0],
                              [-yS, 0, yC, 0],
                              [0, 0, 0, 1]])
  Rotate_Z_matrix = np.array([[zC, -zS, 0, 0],
                              [zS, zC, 0, 0],
                              [0, 0, 1, 0],
                              [0, 0, 0, 1]])
  return np.dot(Rotate_Z_matrix,np.dot(Rotate_Y_matrix,np.dot(Rotate_X_matrix,Translate_matrix)))
