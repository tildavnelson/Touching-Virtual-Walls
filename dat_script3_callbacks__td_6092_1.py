import math
import json
import time

MQTT_TOPIC = 'student/dissertation/tilda/sticks'
STICK_LEN  = 4.0

PIVOTS = {
    's': (0.0,   0.55, -0.8),
    'n': (0.0,   0.7,  -0.8),
    'e': (0.24,  0.31, -0.8),
    'w': (-0.24, 0.31, -0.8)
}

SWING = {'n': 'x', 's': 'x', 'e': 'y', 'w': 'y'}

# Each stick's allowed angle range (degrees, signed, matching the render's rotate).
# The angle swept here IS the angle written to rx/ry — one number, one meaning.
# To reverse a stick's direction, flip its range, e.g. (0,160) <-> (-160,0).
RANGE = {
    's': (0, 160),
    'n': (-160, 0),
    'e': (-160, 0),
    'w': (0, 160),
}

# Walls as bounded rectangles: (axis, transform path, width, height)
WALLS = [
    ('x', '/project1/geo1/transformgrid',  3.0, 2.5),
    ('z', '/project1/geo1/transformgrid1', 3.0, 2.5),
]

SNAP_THRESH    = 0.15
RELEASE_THRESH = 0.25
TIE_EPS        = 0.005

def quat_to_matrix(qw, qx, qy, qz):
    n = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
    if n < 1e-9:
        return [[1,0,0],[0,1,0],[0,0,1]]
    qw, qx, qy, qz = qw/n, qx/n, qy/n, qz/n
    xx=qx*qx; yy=qy*qy; zz=qz*qz
    wx=qw*qx; wy=qw*qy; wz=qw*qz
    xy=qx*qy; xz=qx*qz; yz=qy*qz
    return [
        [1-2*(yy+zz),   2*(xy-wz),   2*(xz+wy)],
        [  2*(xy+wz), 1-2*(xx+zz),   2*(yz-wx)],
        [  2*(xz-wy),   2*(yz+wx), 1-2*(xx+yy)]
    ]

def euler_to_matrix(rx_deg, ry_deg, rz_deg):
    rx = math.radians(rx_deg); ry = math.radians(ry_deg); rz = math.radians(rz_deg)
    cx, sx = math.cos(rx), math.sin(rx)
    cy, sy = math.cos(ry), math.sin(ry)
    cz, sz = math.cos(rz), math.sin(rz)
    Rx = [[1,0,0],[0,cx,-sx],[0,sx,cx]]
    Ry = [[cy,0,sy],[0,1,0],[-sy,0,cy]]
    Rz = [[cz,-sz,0],[sz,cz,0],[0,0,1]]
    def mm(A,B):
        return [[sum(A[i][k]*B[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return mm(mm(Rz, Ry), Rx)   # matches TD's "Rx Ry Rz" rotate order

def apply_mat(M, v):
    return (
        M[0][0]*v[0] + M[0][1]*v[1] + M[0][2]*v[2],
        M[1][0]*v[0] + M[1][1]*v[1] + M[1][2]*v[2],
        M[2][0]*v[0] + M[2][1]*v[1] + M[2][2]*v[2],
    )

def stick_tip_local(sid, angle_deg):
    # Tip of the stick in body-local space for a given SIGNED angle,
    # matching the rotation TD applies (about the swing axis, at the pivot).
    px, py, pz = PIVOTS[sid]
    a = math.radians(angle_deg)
    if SWING[sid] == 'x':
        return (px, py - STICK_LEN*math.sin(a), pz - STICK_LEN*math.cos(a))
    else:
        return (px - STICK_LEN*math.sin(a), py, pz - STICK_LEN*math.cos(a))

def wall_dist(tip_world, walls):
    # true 3D distance from tip to the nearest BOUNDED wall rectangle
    tx_, ty_, tz_ = tip_world
    best = None
    for axis, cx, cy, cz, w, h in walls:
        if axis == 'z':
            dx = max(abs(tx_ - cx) - w/2, 0.0)
            dy = max(abs(ty_ - cy) - h/2, 0.0)
            dz = tz_ - cz
        else:
            dx = tx_ - cx
            dy = max(abs(ty_ - cy) - h/2, 0.0)
            dz = max(abs(tz_ - cz) - w/2, 0.0)
        d = math.sqrt(dx*dx + dy*dy + dz*dz)
        if best is None or d < best:
            best = d
    return best

def onSetupParameters(scriptOp): return
def onPulse(par): return

def onCook(scriptOp):

    #return #this means it's off

    now = time.time()
    last = scriptOp.fetch('_last_pub', 0)
    if (now - last) < 0.15:
        return
    scriptOp.store('_last_pub', now)

    prev_angles = scriptOp.fetch('prev_angles', {'n':0.0,'s':0.0,'e':0.0,'w':0.0})

    scriptOp.clear()
    scriptOp.appendRow(['stick', 'rx', 'ry', 'min_dist'])

    t = op('/project1/sticksval')
    t.clear()
    t.appendRow(['s', 0])
    t.appendRow(['n', 0])
    t.appendRow(['e', 0])
    t.appendRow(['w', 0])
    t.appendRow(['ex', 0])
    t.appendRow(['ey', 0])
    t.appendRow(['ez', 0])

    try:
        # read wall positions live from the actual grid transforms
        walls = []
        for axis, path, w, h in WALLS:
            o = op(path)
            if o is None:
                continue
            walls.append((axis,
                          float(o.par.tx),
                          float(o.par.ty),
                          float(o.par.tz),
                          w, h))

        s   = op('script1')
        lag = op('lag1')

        asm_scale = 0.1

        # world position of the body — EXACTLY as transformtracking places it
        wx = -float(lag['chan1'])
        wy =  float(lag['chan3'])
        wz =  float(lag['chan2'])

        qw = float(s[1,'qw']); qx = float(s[1,'qx'])
        qy = float(s[1,'qy']); qz = float(s[1,'qz'])

        Mraw = quat_to_matrix(qw, qx, qy, qz)
        sy = -Mraw[2][0]
        if abs(sy) < 0.99999:
            ex = math.degrees(math.atan2(Mraw[2][1], Mraw[2][2]))
            ey = math.degrees(math.asin(max(-1.0, min(1.0, sy))))
            ez = math.degrees(math.atan2(Mraw[1][0], Mraw[0][0]))
        else:
            ex = math.degrees(math.atan2(-Mraw[1][2], Mraw[1][1]))
            ey = math.degrees(math.asin(max(-1.0, min(1.0, sy))))
            ez = 0.0

        # same remap the render uses (rx=-ex, ry=ez-90, rz=-ey)
        M = euler_to_matrix(-ex, ez + 90.0, -ey)

        servo_vals = {}

        for sid in PIVOTS:
            lo, hi = RANGE[sid]
            prev = prev_angles.get(sid, 0.0)

            best_ang  = 0.0
            best_dist = None

            step = 5 if hi >= lo else -5
            for ang in range(lo, hi + step, step):
                tl = stick_tip_local(sid, ang)
                tl = (tl[0]*asm_scale, tl[1]*asm_scale, tl[2]*asm_scale)
                rx_, ry_, rz_ = apply_mat(M, tl)
                tip_world = (wx + rx_, wy + ry_, wz + rz_)

                dist = wall_dist(tip_world, walls)
                if dist is None:
                    continue

                if best_dist is None:
                    best_dist = dist
                    best_ang  = ang
                elif dist < best_dist - TIE_EPS:
                    best_dist = dist
                    best_ang  = ang
                elif abs(dist - best_dist) <= TIE_EPS:
                    # tie: prefer the angle closest to where the stick already is
                    if abs(ang - prev) < abs(best_ang - prev):
                        best_ang  = ang
                        best_dist = min(best_dist, dist)

            # hysteresis: looser threshold to release if this stick was already open
            thresh = RELEASE_THRESH if abs(prev) > 1.0 else SNAP_THRESH
            angle = float(best_ang) if (best_dist is not None and best_dist < thresh) else 0.0

            prev_angles[sid] = angle

            rx_out = round(angle, 2) if SWING[sid] == 'x' else 0.0
            ry_out = round(angle, 2) if SWING[sid] == 'y' else 0.0

            t[sid, 1] = round(angle, 2)

            scriptOp.appendRow([sid, rx_out, ry_out,
                round(best_dist, 4) if best_dist is not None else -1])
            servo_vals[sid] = round(abs(angle), 1)

        scriptOp.store('prev_angles', prev_angles)

        scriptOp.appendRow(['EULER', round(ex, 3), round(ey, 3), round(ez, 3)])

        t['ex', 1] = round(ex, 3)
        t['ey', 1] = round(ey, 3)
        t['ez', 1] = round(ez, 3)

        op('mqttclient1').publish(MQTT_TOPIC, json.dumps(servo_vals).encode())

    except Exception as e:
        scriptOp.appendRow(['ERROR', str(e)])

def onGetCookLevel(scriptOp):
    return CookLevel.ALWAYS