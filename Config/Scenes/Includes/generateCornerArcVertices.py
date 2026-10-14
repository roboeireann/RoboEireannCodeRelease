#!/usr/bin/env python3
import math

# --- CONFIGURATION ---
radius_mid = 0.5    # Radius to the center of the line
line_width = 0.05   # Width of the painted line
segments = 16       # Number of quads (16 is smooth for 0.5m)
texture_len = 2.0   # The physical length of your 1024px texture (2.0m)
# ---------------------

r_in = radius_mid - (line_width / 2.0)
r_out = radius_mid + (line_width / 2.0)

# Circumference of 90-degree arc = (pi/2) * radius
arc_length = (math.pi / 2.0) * radius_mid
u_max = arc_length / texture_len

print(f"<!-- Corner Arc: R={radius_mid}, W={line_width} -->")
print("  <Vertices>")
for i in range(segments + 1):
    # Angle from 0 to 90 degrees
    angle = (math.pi / 2.0) * i / segments
    xi, yi = r_in * math.cos(angle), r_in * math.sin(angle)
    xo, yo = r_out * math.cos(angle), r_out * math.sin(angle)
    print(f"    {xi:8.4f} {yi:8.4f} 0    {xo:8.4f} {yo:8.4f} 0")
print("  </Vertices>")

print("\n  <TexCoords>")
for i in range(segments + 1):
    u = u_max * i / segments
    print(f"    {u:8.4f} 0    {u:8.4f} 1")
print("  </TexCoords>")

print("\n  <Quads>")
quad_str = ""
for i in range(segments):
    base = i * 2
    quad_str += f"{base} {base+1} {base+3} {base+2}  "
    if (i + 1) % 4 == 0: quad_str += "\n    "
print(f"    {quad_str.strip()}")
print("  </Quads>")