
import os

filepath = "Sonar.py"
with open(filepath, "r", encoding="utf-8") as f:
    content = f.read()

# 1. Fix visual regression: surface.fill -> draw.circle
# Find the exact line to replace.
old_fill_line = "    # 1. Llenar el fondo con el color seleccionado en el sistema (Azul, Negro, etc.)\n    surface.fill(bg_color)"
new_fill_line = "    # 1. Llenar el fondo con el color seleccionado en el sistema (Azul, Negro, etc.)\n    # Usamos circulo para respetar los margenes del sonar\n    pygame.draw.circle(surface, bg_color, (center_x, center_y), max_radius)"

if old_fill_line in content:
    content = content.replace(old_fill_line, new_fill_line)
else:
    print("Could not find surface.fill line. Checking context...")
    # Fallback search
    if "surface.fill(bg_color)" in content:
        content = content.replace("surface.fill(bg_color)", "pygame.draw.circle(surface, bg_color, (center_x, center_y), max_radius)")
        print("Replaced surface.fill using fallback search.")
    else:
        print("Error: Could not locate background fill instruction.")

# 2. Ensure test_sweep_radius is initialized globally
# We already did this in the previous patch, but let's double check/enforce location if needed.
# The previous patch inserted it before "# --- Performance Optimization Variables ---".
# Let's just verify it exists, or add it if missing (though it should be there).
if "test_sweep_radius = 0" not in content:
    insert_point = "# --- Performance Optimization Variables ---"
    new_init = "test_sweep_radius = 0\n" + insert_point
    content = content.replace(insert_point, new_init)
    print("Added global initialization for test_sweep_radius.")

with open(filepath, "w", encoding="utf-8") as f:
    f.write(content)

print("Successfully patched Sonar.py to fix Eco-1 visuals.")
