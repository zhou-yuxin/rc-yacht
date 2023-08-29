import math
import solid

############################################ configurations #######################################

hull_length = 500
hull_width = 150
hull_height = 60

cabin_head_ratio = 0.05
cabin_tail_ratio = 0.02
cabin_width_ratio = 0.9
cabin_height_ratio = 0.9

keel_airfoil = "0015"
keel_length = 60
keel_length2 = 40
keel_height = 160

drop_length = 150
drop_diameter = 18
drop_thickness = 2

rudder_shaft_offset = 35
rudder_support_shaft_diameter = 5.5
rudder_support_thickness = 2
rudder_support_down = 10

servo_width = 12.5
servo_length = 24
servo_set_width = 5
servo_platform_down = 20
servo_platform_offset = 65
servo_platform_thickness = 3

mast_position = 200
mast_down = 50
mast_diameter = 5.5
mast_support_thickness = 3

sheet_hole_position = 350
sheet_hole_diameter = 3
sheet_hole_angle = 60

rudder_airfoil = "0014"
rudder_length = 40
rudder_length2 = 30
rudder_height = 100
rudder_shaft_diameter = 3.3
rudder_bush_thickness = 1.2
rudder_bush_height = 5

door_width = 100
door_length = 200
door_offset = 20
door_threshold_width = 2
door_threshold_thickness = 1
door_gap = 0.6

door_hook_width = 20
door_hook_length = 8
door_hook_thickness = 1.5

motor_diameter = 10.5
motor_length = 15
motor_wire_diameter = 2
motor_support_thickness = 1.5
motor_support_height = 45
motor_support_angle = 10
motor_support_padding = 10

jib_boom_length = 200
jib_boom_width = 6
jib_boom_height = 5

boom_length = 250
boom_width = 6
boom_height = 5
boom_angle = 0
boom_thickness = 2
boom_diameter = 5.8
boom_set_height = 35

segments = 50

################################### curve resolution of hull shape ################################

w_func = lambda x: math.sin(x * math.pi * 0.65)
h_func = lambda x: math.cos((0.5 * x - 0.25) * math.pi)

###################################### helper functions ###########################################

def naca4(digits, segments, half_cosine_spacing = True, finite_trailing_edge = False):
    m = float(digits[0]) / 100
    p = float(digits[1]) / 10
    t = float(digits[2:]) / 100
    linespace = lambda a, b, n: (a + (b - a) * i / (n - 1) for i in range(n))
    if half_cosine_spacing:
        x = [0.5 * (1 - math.cos(i)) for i in linespace(0, math.pi, segments + 1)]
    else:
        x = list(linespace(0, 1, segments + 1))
    (a0, a1, a2, a3) = (+0.2969, -0.1260, -0.3516, +0.2843)
    a4 = -0.1015 if finite_trailing_edge else -0.1036
    yt = [5 * t * (a0 * math.sqrt(i) + a1 * i + a2 * math.pow(i, 2) + a3 * math.pow(i, 3)   \
        + a4 * math.pow(i, 4)) for i in x]
    if p == 0:
        (xu, xl) = (x, x)
        yu = yt
        yl = [-i for i in yt]
    else:
        xc1 = [i for i in x if i <= p]
        xc2 = [i for i in x if i > p]
        yc1 = [m / math.pow(p, 2) * i * (2 * p - i) for i in xc1]
        yc2 = [m / math.pow(1 - p, 2) * (1 - 2 * p + i) * (1 - i) for i in xc2]
        zc = yc1 + yc2
        dyc1_dx = [m / math.pow(p, 2) * (2 * p - 2 * i) for i in xc1]
        dyc2_dx = [m / math.pow(1 - p, 2) * (2 * p - 2 * i) for i in xc2]
        dyc_dx = dyc1_dx + dyc2_dx
        theta = [math.atan(i) for i in dyc_dx]
        xu = [i - j * math.sin(k) for (i, j, k) in zip(x, yt, theta)]
        yu = [i + j * math.cos(k) for (i, j, k) in zip(zc, yt, theta)]
        xl = [i + j * math.sin(k) for (i, j, k) in zip(x, yt, theta)]
        yl = [i - j * math.cos(k) for (i, j, k) in zip(zc, yt, theta)]
    x = xu[::-1] + xl[1:]
    y = yu[::-1] + yl[1:]
    return (x, y)

def extrude(shape, w_func, h_func, length):
    no0 = lambda x: 0.1 if x == 0 else x
    (last_w, last_h) = (no0(w_func(0)), no0(h_func(0)))
    hull = solid.union()
    for i in range(1, segments + 1):
        r = i / segments
        (w, h) = (w_func(r), h_func(r))
        layer = solid.resize([w, h]).add(shape)
        layer = solid.linear_extrude(height = 1, scale = [last_w / w, last_h / h]).add(layer)
        layer = solid.translate([0, 0, -i]).add(layer)
        hull.add(layer)
        (last_w, last_h) = (w, h)
    hull = solid.scale([1, 1, length / segments]).add(hull)
    hull = solid.rotate([90, 0, 0]).add(hull)
    return hull

###################################### make main body #############################################

def makeHull():
    hull_shape = solid.offset(r = 1, segments = segments).add(
        solid.square([1, 0.5], center = True))
    hull_w_func = lambda x: w_func(x) * hull_width
    hull_h_func = lambda x: 2 * h_func(x) * hull_height
    hull = extrude(shape = hull_shape, w_func = hull_w_func, h_func = hull_h_func,
        length = hull_length)
    cabin_shape = solid.offset(r = 0.8, segments = segments).add(
        solid.square([1, 0.5], center = True))
    cabin_w_func = lambda x: hull_w_func(x) * cabin_width_ratio
    cabin_h_func = lambda x: hull_h_func(x) * cabin_height_ratio
    cabin = extrude(shape = cabin_shape, w_func = cabin_w_func, h_func = cabin_h_func,
        length = hull_length * (1 - cabin_head_ratio - cabin_tail_ratio))
    cabin = solid.translate([0, hull_length * cabin_head_ratio, 0]).add(cabin)
    mask = solid.translate([-hull_width / 2, 0, -hull_height]).add(
        solid.cube([hull_width, hull_length, hull_height]))
    hull = solid.intersection().add(hull).add(mask)
    cabin = solid.intersection().add(cabin).add(mask)
    deck_shape = solid.intersection().add(solid.translate([-1, 0]).add(solid.square([2, 2])))   \
        .add(solid.translate([0, -0.8]).add(solid.circle(r = 1, segments = 2 * segments)))
    deck_w_func = hull_w_func
    deck_h_func = lambda x: math.sin(x * math.pi * 0.9) * hull_height * 0.2
    deck = extrude(shape = deck_shape, w_func = deck_w_func, h_func = deck_h_func,
        length = hull_length)
    sheet_hole = solid.cylinder(d = sheet_hole_diameter, h = hull_height, center = True,
        segments = segments)
    sheet_hole = solid.rotate([sheet_hole_angle, 0, 0]).add(sheet_hole)
    sheet_hole = solid.translate([0, sheet_hole_position, 0]).add(sheet_hole)
    deck -= sheet_hole
    return (hull, cabin, deck)

def makeKeel():
    airfoil = solid.rotate([0, 0, 90]).add(solid.polygon(zip(*naca4(keel_airfoil, segments))))
    airfoil = solid.translate([0, -0.5]).add(airfoil)
    airfoil = solid.resize([0, keel_length], auto = True).add(airfoil)
    height = keel_height + hull_height
    keel = solid.linear_extrude(height = height, scale = keel_length2 / keel_length).add(airfoil)
    points = []
    for i in range(segments + 1):
        y = i / segments
        points.append((math.sin(y * math.pi), y - 0.5))
    shape = solid.resize([drop_diameter / 2, drop_length]).add(solid.polygon(points))
    drop = solid.rotate_extrude(segments = segments).add(shape)
    shape = solid.offset(r = drop_thickness, segments = segments).add(shape)
    mask = solid.translate([0, -drop_length]).add(solid.square([drop_diameter, drop_length * 2]))
    shape = solid.intersection().add(shape).add(mask)
    outer = solid.rotate_extrude(segments = segments).add(shape)
    drop = solid.translate([0, 0, height]).add(solid.rotate([90, 0, 0]).add(drop))
    outer = solid.translate([0, 0, height]).add(solid.rotate([90, 0, 0]).add(outer))
    keel = keel + outer - drop
    keel = solid.mirror([0, 0, 1]).add(keel)
    keel = solid.translate([0, hull_length / 2, 0]).add(keel)
    return keel

def makeRudderSupport():
    width = rudder_support_shaft_diameter + 2 * rudder_support_thickness
    length = rudder_shaft_offset + rudder_support_shaft_diameter / 2 + rudder_support_thickness
    z = -(hull_height + rudder_support_down)
    support = solid.translate([-width / 2, hull_length - length, z]).add(
        solid.cube([width, length, hull_height]))
    shaft = solid.translate([0, hull_length - rudder_shaft_offset, z]).add(
        solid.cylinder(d = rudder_support_shaft_diameter, h = hull_height, segments = segments))
    return (support, shaft)

def makeServoPlatform():
    platform = solid.square([hull_width, servo_length + 2 * servo_set_width], center = True)
    servo1 = solid.square([servo_width, servo_length], center = True)
    servo2 = solid.translate([-hull_width / 4, 0, 0]).add(servo1)
    servo3 = solid.translate([hull_width / 4, 0, 0]).add(servo1)
    platform -= servo1 + servo2 + servo3
    platform = solid.linear_extrude(height = servo_platform_thickness).add(platform)
    platform = solid.translate([0, hull_length - servo_platform_offset,
        -(servo_platform_down + servo_platform_thickness)]).add(platform)
    return platform

def makeMastSupport():
    size = mast_diameter + 2 * mast_support_thickness
    support = solid.translate([-size / 2, -size / 2 + mast_position, -hull_height]).add(
        solid.cube([size, size, hull_height]))
    mast = solid.translate([0, mast_position, -mast_down]).add(
        solid.cylinder(d = mast_diameter, h = hull_height * 2, segments = segments))
    return (support, mast)

def makeDoor():
    position = hull_length - door_offset - door_length
    diff1 = solid.translate([-door_width / 2, position, door_threshold_thickness]).add(
        solid.cube([door_width, door_length, hull_height]))
    diff2 = solid.translate([-door_width / 2 + door_threshold_width,
        position + door_threshold_width, 0]).add(
        solid.cube([door_width - 2 * door_threshold_width, door_length - 2 * door_threshold_width,
        hull_height]))
    diff = diff1 + diff2
    mask = solid.translate([-door_width / 2 + door_gap, position + door_gap,
        door_threshold_thickness]).add(solid.cube([door_width - 2 * door_gap,
        door_length - 2 * door_gap, hull_height]))
    return (diff, mask)

def makeDoorHook():
    hook = solid.cube([door_hook_width, door_hook_length,
        door_hook_thickness + door_threshold_thickness + door_gap])
    diff = solid.cube([door_hook_width, door_hook_length - door_hook_thickness,
        door_threshold_thickness + door_gap])
    hook = solid.mirror([0, 0, 1]).add(hook - diff)
    pos = hull_length - door_offset - door_length + door_gap + door_threshold_width \
        + door_hook_thickness - door_hook_length
    hook = solid.translate([-door_hook_width / 2, pos, door_threshold_thickness]).add(hook)
    return hook

def makeMotorSupport():
    h = motor_length + motor_support_thickness
    cylinder = solid.cylinder(d = motor_diameter + 2 * motor_support_thickness, h = h,
        segments = segments)
    w = motor_wire_diameter + 2 * motor_support_thickness
    cube = solid.translate([-w / 2, 0, 0]).add(solid.cube([w, motor_support_height, h]))
    motor = solid.translate([0, 0, motor_support_thickness]).add(
        solid.cylinder(d = motor_diameter, h = motor_length, segments = segments))
    support = cylinder + cube - motor
    wire = solid.cube([motor_wire_diameter, motor_support_height + hull_height,
        motor_length - motor_support_thickness])
    wire = solid.translate([-motor_wire_diameter / 2, 0, motor_support_thickness]).add(wire)
    def move(obj):
        obj = solid.translate([0, -motor_support_height, 0]).add(obj)
        obj = solid.rotate([-90, motor_support_angle, 0]).add(obj)
        obj = solid.translate([door_width / 2 - motor_support_padding,
            hull_length - door_offset - motor_support_padding - motor_length,
            w / 2 * math.sin(math.radians(motor_support_angle)) + door_threshold_thickness])    \
            .add(obj)
        return obj
    support = move(support)
    wire = move(wire)
    return (support, wire)

def assembleBody():
    (hull, cabin, deck) = makeHull()
    keel = makeKeel()
    (rudder_support, rudder_shaft) = makeRudderSupport()
    servo_platform = makeServoPlatform()
    (mast_support, mast) = makeMastSupport()
    (door_diff, door_mask) = makeDoor()
    hull = hull + deck + keel   \
        - (cabin - rudder_support - servo_platform - mast_support)  \
        - rudder_shaft - mast - door_diff
    door_hook = makeDoorHook()
    (motor_support1, motor_wire1) = makeMotorSupport()
    motor_support2 = solid.mirror([1, 0, 0]).add(motor_support1)
    motor_wire2 = solid.mirror([1, 0, 0]).add(motor_wire1)
    door = solid.intersection().add(deck).add(door_mask) + door_hook    \
        + motor_support1 + motor_support2 - motor_wire1 - motor_wire2
    return (hull, door)

######################################## make parts ###############################################

def makeRudder():
    airfoil = solid.rotate([0, 0, 90]).add(solid.polygon(zip(*naca4(rudder_airfoil, segments))))
    airfoil = solid.translate([0, -0.4]).add(airfoil)
    airfoil = solid.resize([0, rudder_length], auto = True).add(airfoil)
    rudder = solid.linear_extrude(height = rudder_height,
        scale = rudder_length2 / rudder_length).add(airfoil)
    rudder = solid.mirror([0, 0, 1]).add(rudder)
    bush = solid.cylinder(d = rudder_shaft_diameter + 2 * rudder_bush_thickness,
        h = rudder_bush_height, segments = segments)
    shaft = solid.cylinder(d = rudder_shaft_diameter, h = rudder_height / 3 + rudder_bush_height,
        segments = segments)
    shaft = solid.translate([0, 0, -rudder_height / 4]).add(shaft)
    rudder = rudder + bush - shaft
    return rudder

def makeJibBoom():
    boom = solid.cube([jib_boom_width, jib_boom_length, jib_boom_height])
    boom = solid.translate([-jib_boom_width / 2, 0, 0]).add(boom)
    end1 = solid.cylinder(d = jib_boom_width, h = jib_boom_height, segments = segments)
    end2 = solid.translate([0, jib_boom_length, 0]).add(end1)
    boom += end1 + end2
    return boom

def makeBoom():
    boom = solid.cube([boom_width, boom_length, boom_height])
    boom = solid.translate([-boom_width / 2, 0, 0]).add(boom)
    end = solid.cylinder(d = boom_width, h = boom_height, segments = segments)
    end = solid.translate([0, boom_length, 0]).add(end)
    boom += end
    boom = solid.rotate([boom_angle, 0, 0]).add(boom)
    angle = math.radians(boom_angle)
    boom = solid.translate([0, 0, boom_set_height - (boom_height / math.cos(angle) + 
        (mast_diameter / 2 + boom_thickness) * math.tan(angle))]).add(boom)
    set = solid.cylinder(d = mast_diameter + 2 * boom_thickness, h = boom_set_height,
        segments = segments)
    diff = solid.cylinder(d = boom_diameter, h = boom_set_height, segments = segments)
    boom = boom + set - diff
    return boom

############################################ test #################################################

def test():
    (hull, door) = assembleBody()
    solid.scad_render_to_file(hull, "hull.scad")
    solid.scad_render_to_file(door, "door.scad")
    rudder = makeRudder()
    solid.scad_render_to_file(rudder, "rudder.scad")
    jib_boom = makeJibBoom()
    solid.scad_render_to_file(jib_boom, "jib_boom.scad")
    boom = makeBoom()
    solid.scad_render_to_file(boom, "boom.scad")
    door = solid.translate([0, 0, hull_height]).add(door)
    rudder = solid.translate([0, hull_length - rudder_shaft_offset, -hull_height]).add(rudder)
    jib_boom = solid.translate([0, mast_position - jib_boom_length - 20, 2 * hull_height])  \
        .add(jib_boom)
    boom = solid.translate([0, mast_position, 2 * hull_height]).add(boom)
    overview = hull + door + rudder + jib_boom + boom
    solid.scad_render_to_file(overview, "overview.scad")

if __name__ == "__main__":
    test()
