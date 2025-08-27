import numpy as np
import math 

L0 = 0.04
L1 = 0.12
L2 = 0.165

fx = 570.3405082258201
fy = 570.3405082258201
cx = 319.5
cy = 239.5

T_cam_to_robot = np.array([
    [-1, 0,  0, 0],   
    [0,  1,  0, 0.27],    
    [0,  0,  -1, -0.24], 
    [0,  0,  0, 1]
])

# T_cam_to_robot = np.array([
#     [0, -1,  0, 0.27],   
#     [1,  0,  0, 0],    
#     [0,  0,  -1, 0.245], 
#     [0,  0,  0, 1]
# ])

def pixel_to_camera(u, v, z):
    u = 640 - u 
    v = 480 - v 
    x = (u - cx) * z / fx 
    y = (v - cy) * z / fy
    z = z
    return x, y, z 

def camera_to_robot(xc, yc, zc):
    cam_point = np.array([xc, yc, zc, 1])
    robot_point = T_cam_to_robot @ cam_point
    return robot_point[:3] 


# A1, B1 = 1.727412, -4.679241   # s1 = A1*t1 + B1
# A2, B2 = 1.877638, -11.741374  # s2 = A2*t2 + B2
# A3, B3 = -3.388963, -60.951913 # s3 = A3*t3 + B3

# def ik_angles_to_servo_angles(t1, t2, t3, clamp=None, radians=False):
#     import math
#     if radians:
#         t1, t2, t3 = map(math.degrees, (t1, t2, t3))

#     s1 = A1*t1 + B1
#     s2 = A2*t2 + B2
#     s3 = A3*t3 + B3

#     if clamp is not None:
#         (mn1, mx1), (mn2, mx2), (mn3, mx3) = clamp
#         s1 = max(mn1, min(mx1, s1))
#         s2 = max(mn2, min(mx2, s2))
#         s3 = max(mn3, min(mx3, s3))

#     return s1, s2, s3
# COEF = {
#     "s1": {"b0":-22.0563592424,"bt":0.2722190978,"bx":0.1643545261,"by":0.4691398622,
#            "bxx":-0.00018037,"byy":-0.00008612,"bxy":-0.00017256,"bxt":-0.00257856,"byt":-0.00530782},
#     "s2": {"b0":-1.9655907756,"bt":0.7479014565,"bx":-0.3239534465,"by":-0.0877343174,
#            "bxx":-0.00014979,"byy":0.00000105,"bxy":-0.00025282,"bxt":-0.00482237,"byt":0.00026361},
#     "s3": {"b0":-13.6966504633,"bt":1.4619560227,"bx":0.0173276259,"by":0.5250354881,
#            "bxx":-0.00019503,"byy":0.00164889,"bxy":-0.00008395,"bxt":-0.00109232,"byt":-0.03799485},
# }

# def _phi_terms(x, y, t, cx=cx, cy=cy):
#     xp, yp = x - cx, y - cy
#     return xp, yp, xp*xp, yp*yp, xp*yp, xp*t, yp*t

# def pixel_t_to_servo(x, y, t1, t2, t3):
#     # s1
#     xp, yp, x2, y2, xy, xt, yt = _phi_terms(x, y, t1)
#     c = COEF["s1"]
#     s1 = (c["b0"] + c["bt"]*t1 + c["bx"]*xp + c["by"]*yp +
#           c["bxx"]*x2 + c["byy"]*y2 + c["bxy"]*xy + c["bxt"]*xt + c["byt"]*yt)

#     # s2
#     xp, yp, x2, y2, xy, xt, yt = _phi_terms(x, y, t2)
#     c = COEF["s2"]
#     s2 = (c["b0"] + c["bt"]*t2 + c["bx"]*xp + c["by"]*yp +
#           c["bxx"]*x2 + c["byy"]*y2 + c["bxy"]*xy + c["bxt"]*xt + c["byt"]*yt)

#     # s3
#     xp, yp, x2, y2, xy, xt, yt = _phi_terms(x, y, t3)
#     c = COEF["s3"]
#     s3 = (c["b0"] + c["bt"]*t3 + c["bx"]*xp + c["by"]*yp +
#           c["bxx"]*x2 + c["byy"]*y2 + c["bxy"]*xy + c["bxt"]*xt + c["byt"]*yt)

#     return float(s1), float(s2), float(s3)

COEF = {
    "s1": {"b0": 0.001934744, "bt": 0.023276711, "bx": 0.256175549, "by": -0.158015115,
           "bxx": -0.000259226, "byy": -0.000143003, "bxy": 0.000231815, "bxt": -0.003882440, "byt": 0.001529852},
    "s2": {"b0": -0.014461861, "bt": 0.775668373, "bx": -0.305427010, "by": -0.439576681,
           "bxx": -0.000151588, "byy": -0.000231221, "bxy": -0.000257492, "bxt": -0.004542900, "byt": -0.004960016},
    "s3": {"b0": 0.083243846, "bt": 0.877120174, "bx": -0.157188215, "by": -1.310337019,
           "bxx": -0.000305855, "byy": -0.002194977, "bxy": -0.000354970, "bxt": 0.007258964, "byt": 0.044603038},
}

def _terms(x, y, t):
    xp, yp = x - cx, y - cy
    return xp, yp, xp*xp, yp*yp, xp*yp, xp*t, yp*t

def pixel_t_to_servo(x, y, t1, t2, t3):
    # s1
    xp, yp, x2, y2, xy, xt, yt = _terms(x, y, t1)
    c = COEF["s1"]
    s1 = (c["b0"] + c["bt"]*t1 + c["bx"]*xp + c["by"]*yp +
          c["bxx"]*x2 + c["byy"]*y2 + c["bxy"]*xy + c["bxt"]*xt + c["byt"]*yt)
    # s2
    xp, yp, x2, y2, xy, xt, yt = _terms(x, y, t2)
    c = COEF["s2"]
    s2 = (c["b0"] + c["bt"]*t2 + c["bx"]*xp + c["by"]*yp +
          c["bxx"]*x2 + c["byy"]*y2 + c["bxy"]*xy + c["bxt"]*xt + c["byt"]*yt)
    # s3
    xp, yp, x2, y2, xy, xt, yt = _terms(x, y, t3)
    c = COEF["s3"]
    s3 = (c["b0"] + c["bt"]*t3 + c["bx"]*xp + c["by"]*yp +
          c["bxx"]*x2 + c["byy"]*y2 + c["bxy"]*xy + c["bxt"]*xt + c["byt"]*yt)
    return float(s1), float(s2), float(s3)

def inverse_kinematics(u, v, z=0.215, L0=L0, L1=L1, L2=L2, phi_deg=-45, return_radians=False):
    x, y, z = pixel_to_camera(u, v, z)
    x, y, z = camera_to_robot(x, y, z)
    
    phi = np.radians(phi_deg)

    t1 = np.arctan2(y, x) 
    nx = x * np.cos(t1) + y * np.sin(t1)
    ny = z - L0

    wx = nx - L2 * np.cos(phi) 
    wy = ny - L2 * np.sin(phi)

    c2 = (wx**2 + wy**2 - L1**2 - L2**2) / (2 * L1 * L2)
    c2 = np.clip(c2, -1.0, 1.0)
    s2 = np.sqrt(1 - c2**2)

    t2 = np.arctan2(wy, wx) - np.arctan2(L2 * s2, L1 + L2 * c2)

    t3 = phi - t2

    if return_radians:
        return (t1, t2, t3)

    sol_deg = tuple(np.degrees(a) for a in (t1, t2, t3))
    t1, t2, t3 = tuple(a for a in sol_deg)

    print(u, v)
    print(t1, t2, t3)
    t1, t2, t3 = pixel_t_to_servo(u, v, t1, t2, t3)
    if u < 360:
        if v < 155:
            t1 += 3.5
            t2 += 5
            t3 -= 10
        else: 
            if u < 150: 
                t3 -= 10
            else: 
                t1 += 2.5 
                t3 -= 15
    else:
        t1 -= 2.5
        t3 -= 15
    return t1, t2, t3