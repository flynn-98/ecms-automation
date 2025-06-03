import pickle

adding_side_holes = False
tcp_lenght = 18.1
p1 = [495.67, -141.07, -22.99 + tcp_lenght]
p1_prime = [575.0, -125.00, tcp_lenght]
# 3x12 ( x, y)

p2 = [36.14, -530.20, -22.23 + tcp_lenght]
p2_prime = [525.0, -725.0, tcp_lenght]
# 15x11

p3 = [-105.44, -388.90, -22.15 + tcp_lenght]
p3_prime = [325.0, -725.0, tcp_lenght]
# 15x7

p4 = [319.03, -388.72, -22.35 + tcp_lenght]
p4_prime = [625.0, -425.0, tcp_lenght]
# 9x13

# (measured side of table, need to consider tool length on x axis)
p5 = [522.30, 27.17, -60.28]
p5_prime = [475.0, 0 - tcp_lenght, -25.0]
# 0x10x-1

# (measured side of table, need to consider tool length on x axis)
p6 = [-25.60, -592.35, -59.68]
p6_prime = [525.0, -775.0 + tcp_lenght, 25.0]
# 16x11x-1

p7 = [672.90, -388.43, -22.48 + tcp_lenght]
p7_prime = [875.0, -175.0, tcp_lenght]
# 4x18

p8 = [213.15, -636.18, -21.60 + tcp_lenght]
p8_prime = [725.0, -675.0, tcp_lenght]
# 14x15

p9 = [460.62, -388.49, -22.76 + tcp_lenght]
p9_prime = [725.0, -325.0, tcp_lenght]
# 7x15

# p1 = [495.67, -141.07, -22.99-tcp_lenght]
# p1_prime=[125.00, 575.0, 0]
# # 3x12 ( x, y)

# p2 = [36.14, -530.20, -22.23-tcp_lenght]
# p2_prime=[725.0, 525.0, 0]
# # 15x11

# p3=[-105.44, -388.90, -22.15-tcp_lenght]
# p3_prime=[725.0, 325.0, 0]
# # 15x7

# p4=[319.03, -388.72, -22.35-tcp_lenght]
# p4_prime=[425.0, 625.0, 0]
# # 9x13

# #(measured side of table, need to consider tool length on x axis)
# p5=[522.30, 27.17, -60.28]
# p5_prime=[0-tcp_lenght, 475.0, -25.0]
# # 0x10x-1

# #(measured side of table, need to consider tool length on x axis)
# p6=[-25.60, -592.35, -59.68]
# p6_prime=[775.0+tcp_lenght, 525.0, -25.0]
# # 16x11x-1

# p7=[672.90, -388.43, -22.48-tcp_lenght]
# p7_prime=[175.0, 875.0, 0]
# # 4x18

# p8=[213.15, -636.18, -21.60-tcp_lenght]
# p8_prime=[675.0, 725.0, 0]
# # 14x15

# p9 = [460.62, -388.49, -22.76-tcp_lenght]
# p9_prime = [325.0, 725.0, 0]
# # 7x15

if not adding_side_holes:
    p = [p1, p2, p3, p4, p7, p8, p9]  # robot base p5, p6,
    p_prime = [
        p1_prime,
        p2_prime,
        p3_prime,
        p4_prime,
        p7_prime,
        p8_prime,
        p9_prime,
    ]  # world base  p5_prime, p6_prime
# else:
#     p = [p1, p2, p3, p4, p5, p6, p7, p8, p9]  # robot base p5, p6,
#     p_prime = [
#         p1_prime,
#         p2_prime,
#         p3_prime,
#         p4_prime,
#         p5_prime,
#         p6_prime,
#         p7_prime,
#         p8_prime,
#         p9_prime,
#     ]  # world base  p5_prime, p6_prime,

# save datas as pickel
with open("robot_base_p.pkl", "wb") as fp:
    pickle.dump(p, fp)

with open("world_base_p_prime.pkl", "wb") as fp:
    pickle.dump(p_prime, fp)
