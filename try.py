# t = 250
# k = 6/100
# Kp = 4
# Ti = 30/4
# T0 = 5
# SP = 31
# PV = 26
# Ui = 0
# sat = None
#
#
# for i in range(100):
#     e = SP-PV
#     Up = Kp * e
#     if sat == 0:
#         Ui = Ui + Kp * e * T0/Ti
#     U = Up + Ui
#     print(U)
#
#     if U > 100:
#         U = 100
#         sat = 1
#     elif U < 0:
#         U = 0
#         sat = 1
#     else:
#         sat = 0

t = 250
k = 6 / 100
Kp = 4
Ti = 30 / 4
T0 = 5
SP = 31
Ui = 0
sat = 0


# t  = (sensor.temperature)
def control(value):
    global sat
    global Ui
    e = SP - value
    Up = Kp * e
    if sat == 0:
        Ui = Ui + Kp * e * T0 / Ti
    U = Up + Ui

    if U > 100:
        U = 100
        sat = 1
    elif U < 0:
        U = 0
        sat = 1
    else:
        sat = 0
    return U
a = control(26)
print(a)
print(Ui)
print(sat)