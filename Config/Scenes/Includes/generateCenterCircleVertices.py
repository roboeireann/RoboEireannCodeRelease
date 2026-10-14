#!/usr/bin/env python3
import math

# Constants for a 3m diameter circle with 0.05m lines
r_in = 1.475
r_out = 1.525
segments = 32
texture_tile_factor = 2.35619  # (pi * 1.5m) / 2m texture length

print("  <Vertices>")

for i in range(segments + 1):
    # Angle from -pi/2 (-90 deg) to +pi/2 (+90 deg)
    angle = -math.pi/2 + (math.pi * i / segments)
    
    # Standard Trig: X is the peak (cos), Y is the side (sin)
    xi, yi = r_in * math.cos(angle), r_in * math.sin(angle)
    xo, yo = r_out * math.cos(angle), r_out * math.sin(angle)
    
    print(f"{xi:8.4f} {yi:8.4f} 0    {xo:8.4f} {yo:8.4f} 0")

print("\n  </Vertices>")

print("  <TexCoords>")

for i in range(segments + 1):
    u = texture_tile_factor * i / segments
    print(f"{u:8.4f} 0    {u:8.4f} 1")

print("  </TexCoords>")
