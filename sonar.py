# Importamos las bibliotecas pygame y math
import pygame
import math
import os # Needed for path manipulation
import serial # Already present
import serial.tools.list_ports # For COM port detection
from pygame.locals import *
from geopy.distance import geodesic
from geopy.point import Point


# Inicializamos el motor de juegos
pygame.init()
pygame.mixer.init() # Initialize the mixer explicitly

# --- COM Port Detection Utility ---
def get_available_com_ports():
    ports = serial.tools.list_ports.comports()
    available_ports = [port.device for port in ports]
#print(f"Available COM ports: {available_ports}") # For debugging
    return available_ports
# --- End COM Port Detection Utility ---

# Updated initial serial state:
puerto = None  # Will store the currently connected port device string
baudios = 9600  # Default baud rate, can be changed by user
ser = None
serial_port_available = False # Will be True only after a successful connection by user


# Initialize NMEA data variables
latitude_str = "N/A"
longitude_str = "N/A"
current_ship_lat_deg = None # Numerical latitude in decimal degrees
current_ship_lon_deg = None # Numerical longitude in decimal degrees

speed_str = "N/A"
heading_str = "N/A"
current_ship_heading = 0.0  # Initialize current ship heading
zda_time_str = "N/A"
zda_date_str = "N/A"
# rot_str = "N/A" # ROT string removed

att_pitch_str = "N/A"
att_roll_str = "N/A"

# Helper function to convert NMEA lat/lon (DDDMM.MMMM, H) to decimal degrees
def nmea_to_decimal_degrees(nmea_val_str, hemisphere):
    """Converts NMEA format latitude or longitude to decimal degrees."""
    if not nmea_val_str or not hemisphere:
        return None
    try:
        val = float(nmea_val_str)
        degrees = int(val / 100)
        minutes = val % 100
        decimal_degrees = degrees + (minutes / 60.0)
        if hemisphere in ['S', 'W']:
            decimal_degrees *= -1
        return decimal_degrees
    except ValueError:
        print(f"Error converting NMEA value {nmea_val_str} to float.")
        return None

# Helper function to format minutes to three decimal places
def format_minutes_to_3dp(raw_minutes_str):
    if '.' in raw_minutes_str:
        parts = raw_minutes_str.split('.', 1)
        integer_part = parts[0]
        decimal_part = parts[1] if len(parts) > 1 else ""

        # Ensure decimal_part is handled correctly if it's empty after split (e.g. "47.")
        # Pad with zeros or truncate to get 3 decimal places
        formatted_decimals = (decimal_part + "000")[:3]
        return f"{integer_part}.{formatted_decimals}"
    else:
        # Input is a whole number, e.g., "47"
        return f"{raw_minutes_str}.000"

def parse_zda(sentence):
    global zda_time_str, zda_date_str
    try:
        parts = sentence.split(',')
        if len(parts) > 4:
            utc_time_full = parts[1] # e.g., "172814.12"
            day = parts[2]
            month = parts[3]
            year = parts[4]

            # Format time: HH:MM:SS
            if len(utc_time_full) >= 6:
                h = utc_time_full[0:2]
                m = utc_time_full[2:4]
                s = utc_time_full[4:6]
                zda_time_str = f"{h}:{m}:{s}"
            else:
                zda_time_str = "N/A" # Or keep old value

            # Format date: dd/mm/yyyy
            if day and month and year: # Ensure all parts are present
                zda_date_str = f"{day}/{month}/{year}"
            else:
                zda_date_str = "N/A" # Or keep old value
        # else: fields not present, keep old values or set to N/A
    except Exception as e:
        print(f"Error parsing ZDA sentence: {sentence} - {e}")

def parse_rot(sentence): # This function is no longer called but kept for now
    global rot_str
    try:
        parts = sentence.split(',')
        if len(parts) > 2: 
            status = parts[2].strip().upper()
            if status == 'A':
                rate_of_turn_field = parts[1] 
                if not rate_of_turn_field:
                    rot_str = "N/A"
                    return
                try:
                    rate_of_turn_val = float(rate_of_turn_field)
                    if rate_of_turn_val >= 0:
                        rot_str = f"{rate_of_turn_val:.1f} BABOR"
                    else:
                        rot_str = f"{abs(rate_of_turn_val):.1f} ESTRIBOR"
                except ValueError:
                    rot_str = "N/A"
            else:
                rot_str = "N/A"
        else:
            rot_str = "N/A"
    except Exception as e:
        print(f"Generic error parsing ROT sentence: {sentence} - {e}")
        rot_str = "N/A"

def parse_fec_gpatt(sentence):
    global att_pitch_str, att_roll_str
    try:
        parts = sentence.split(',')
        if len(parts) > 4: 
            pitch_val_str = parts[3]
            roll_val_full_str = parts[4] 
            roll_val_str = roll_val_full_str.split('*')[0]
            if pitch_val_str:
                try:
                    float(pitch_val_str) 
                    att_pitch_str = f"{pitch_val_str}°"
                except ValueError:
                    att_pitch_str = "N/A" 
                    print(f"Invalid pitch value: {pitch_val_str} in {sentence}")
            else:
                att_pitch_str = "N/A"
            if roll_val_str: 
                try:
                    float(roll_val_str) 
                    att_roll_str = f"{roll_val_str}°"
                except ValueError:
                    att_roll_str = "N/A" 
                    print(f"Invalid roll value: {roll_val_str} in {sentence}")
            else:
                att_roll_str = "N/A"
    except Exception as e:
        print(f"Error parsing PFEC,GPatt sentence: {sentence} - {e}")

def parse_gll(sentence):
    global latitude_str, longitude_str, current_ship_lat_deg, current_ship_lon_deg
    try:
        parts = sentence.split(',')
        if len(parts) > 6 and parts[6].upper() == 'V': 
            return
        if len(parts) > 4 and parts[1] and parts[2] and parts[3] and parts[4]:
            lat_nmea_val = parts[1]
            lat_hemisphere = parts[2]
            lon_nmea_val = parts[3]
            lon_hemisphere = parts[4]
            if len(lat_nmea_val) >= 4:
                degrees = lat_nmea_val[:2]
                raw_minutes = lat_nmea_val[2:]
                formatted_minutes = format_minutes_to_3dp(raw_minutes)
                latitude_str = f"{degrees}° {formatted_minutes}{lat_hemisphere}"
            else:
                latitude_str = f"{lat_nmea_val} {lat_hemisphere}"
            if len(lon_nmea_val) >= 5:
                degrees = lon_nmea_val[:3]
                raw_minutes = lon_nmea_val[3:]
                formatted_minutes = format_minutes_to_3dp(raw_minutes)
                longitude_str = f"{degrees}° {formatted_minutes}{lon_hemisphere}"
            else:
                longitude_str = f"{lon_nmea_val} {lon_hemisphere}"
            current_ship_lat_deg = nmea_to_decimal_degrees(lat_nmea_val, lat_hemisphere)
            current_ship_lon_deg = nmea_to_decimal_degrees(lon_nmea_val, lon_hemisphere)
            if current_ship_lat_deg is None: latitude_str = "N/A (Parse Err)"
            if current_ship_lon_deg is None: longitude_str = "N/A (Parse Err)"
    except (IndexError, ValueError) as e:
        print(f"Error parsing GLL sentence: {sentence} - {e}")
        latitude_str = "N/A (Exception)"
        longitude_str = "N/A (Exception)"
        current_ship_lat_deg = None
        current_ship_lon_deg = None

def parse_gga(sentence):
    global latitude_str, longitude_str, current_ship_lat_deg, current_ship_lon_deg
    try:
        parts = sentence.split(',')
        if len(parts) > 6 and parts[6] == '0': 
            return
        if len(parts) > 5 and parts[2] and parts[3] and parts[4] and parts[5]:
            lat_nmea_val = parts[2]
            lat_hemisphere = parts[3]
            lon_nmea_val = parts[4]
            lon_hemisphere = parts[5]
            if len(lat_nmea_val) >= 4:
                degrees = lat_nmea_val[:2]
                raw_minutes = lat_nmea_val[2:]
                formatted_minutes = format_minutes_to_3dp(raw_minutes)
                latitude_str = f"{degrees}° {formatted_minutes}{lat_hemisphere}"
            else:
                latitude_str = f"{lat_nmea_val} {lat_hemisphere}"
            if len(lon_nmea_val) >= 5:
                degrees = lon_nmea_val[:3]
                raw_minutes = lon_nmea_val[3:]
                formatted_minutes = format_minutes_to_3dp(raw_minutes)
                longitude_str = f"{degrees}° {formatted_minutes}{lon_hemisphere}"
            else:
                longitude_str = f"{lon_nmea_val} {lon_hemisphere}"
            current_ship_lat_deg = nmea_to_decimal_degrees(lat_nmea_val, lat_hemisphere)
            current_ship_lon_deg = nmea_to_decimal_degrees(lon_nmea_val, lon_hemisphere)
            if current_ship_lat_deg is None: latitude_str = "N/A (Parse Err)"
            if current_ship_lon_deg is None: longitude_str = "N/A (Parse Err)"
    except (IndexError, ValueError) as e:
        print(f"Error parsing GGA sentence: {sentence} - {e}")
        latitude_str = "N/A (Exception)"
        longitude_str = "N/A (Exception)"
        current_ship_lat_deg = None
        current_ship_lon_deg = None

def parse_rmc(sentence):
    global latitude_str, longitude_str, current_ship_lat_deg, current_ship_lon_deg
    try:
        parts = sentence.split(',')
        if len(parts) > 2 and parts[2].upper() == 'V':
            return
        if len(parts) > 6 and parts[3] and parts[4] and parts[5] and parts[6]:
            lat_nmea_val = parts[3]
            lat_hemisphere = parts[4]
            lon_nmea_val = parts[5]
            lon_hemisphere = parts[6]
            if len(lat_nmea_val) >= 4:
                degrees = lat_nmea_val[:2]
                raw_minutes = lat_nmea_val[2:]
                formatted_minutes = format_minutes_to_3dp(raw_minutes)
                latitude_str = f"{degrees}° {formatted_minutes}{lat_hemisphere}"
            else:
                latitude_str = f"{lat_nmea_val} {lat_hemisphere}"
            if len(lon_nmea_val) >= 5:
                degrees = lon_nmea_val[:3]
                raw_minutes = lon_nmea_val[3:]
                formatted_minutes = format_minutes_to_3dp(raw_minutes)
                longitude_str = f"{degrees}° {formatted_minutes}{lon_hemisphere}"
            else:
                longitude_str = f"{lon_nmea_val} {lon_hemisphere}"
            current_ship_lat_deg = nmea_to_decimal_degrees(lat_nmea_val, lat_hemisphere)
            current_ship_lon_deg = nmea_to_decimal_degrees(lon_nmea_val, lon_hemisphere)
            if current_ship_lat_deg is None: latitude_str = "N/A (Parse Err)"
            if current_ship_lon_deg is None: longitude_str = "N/A (Parse Err)"
    except (IndexError, ValueError) as e:
        print(f"Error parsing RMC sentence: {sentence} - {e}")
        latitude_str = "N/A (Exception)"
        longitude_str = "N/A (Exception)"
        current_ship_lat_deg = None
        current_ship_lon_deg = None

def parse_vtg(sentence):
    global speed_str
    try:
        parts = sentence.split(',')
        if len(parts) > 5 and parts[5]:
            speed_str = f"{parts[5]} Knots"
    except (IndexError, ValueError) as e:
        print(f"Error parsing VTG sentence: {sentence} - {e}")

def parse_hdt(sentence):
    global heading_str, current_ship_heading
    try:
        parts = sentence.split(',')
        if len(parts) > 1 and parts[1]:
            raw_heading = parts[1]
            heading_str = f"{raw_heading} T"
            try:
                current_ship_heading = float(raw_heading)
            except ValueError:
                print(f"Error converting HDT heading to float: {raw_heading} in {sentence}")
    except Exception as e:
        print(f"Error parsing HDT sentence: {sentence} - {e}")

def parse_hdg(sentence):
    global heading_str, current_ship_heading
    try:
        parts = sentence.split(',')
        if len(parts) > 1 and parts[1]: 
            raw_hdg_value = parts[1]
            try:
                current_ship_heading = float(raw_hdg_value)
                heading_display_text = f"{raw_hdg_value}"
                if len(parts) > 4 and parts[3] and parts[4]: 
                    heading_display_text += f" (Mag Var: {parts[3]} {parts[4]})"
                else:
                    heading_display_text += " (No Mag Var)"
                heading_str = heading_display_text
            except ValueError:
                print(f"Error converting HDG heading to float: {raw_hdg_value} in {sentence}")
                heading_display_text = f"{raw_hdg_value}"
                if len(parts) > 4 and parts[3] and parts[4]:
                    heading_display_text += f" (Mag Var: {parts[3]} {parts[4]})"
                else:
                    heading_display_text += " (No Mag Var)"
                heading_str = heading_display_text
    except Exception as e:
        print(f"Error parsing HDG sentence: {sentence} - {e}")
try:
    display_info = pygame.display.Info()
    # initial_width = display_info.current_w
    # initial_height = display_info.current_h
    # Opcional: Usar un porcentaje del tamaño del escritorio o un máximo
    initial_width = int(display_info.current_w * 0.9)
    initial_height = int(display_info.current_h * 0.9)
except pygame.error:
    initial_width = 1024 # Fallback
    initial_height = 768  # Fallback

dimensiones = [initial_width, initial_height]
screen = pygame.display.set_mode(dimensiones, pygame.RESIZABLE)
# Asegúrate de que las líneas de set_caption y la asignación a `pantalla` permanezcan como estaban.




# Definimos algunos colores
# Estos se convertirán en los colores por defecto (esquema 3)
DEFAULT_NEGRO = (0, 0, 0)
DEFAULT_BLANCO = (255, 255, 255)
DEFAULT_VERDE = (0, 255, 0)
DEFAULT_ROJO = (255, 0, 0)
DEFAULT_AZUL = (0, 0, 255) # Color de fondo principal para el esquema por defecto
DEFAULT_CELESTE = (173, 216, 230) # Light Blue for sweep
DEFAULT_VERDE_CLARO = (144, 238, 144) # Light Green for some square selectors
DEFAULT_GRIS_MEDIO = (128, 128, 128)
DEFAULT_GRIS_MUY_CLARO = (220, 220, 220)

# --- Definiciones de Esquemas de Color ---
color_schemes = {
    1: { # Verde Militar
        "BACKGROUND": (47, 79, 47),        # Verde oscuro (Dark Olive Green-ish)
        "PRIMARY_TEXT": (188, 238, 104),   # Verde amarillento claro (tipo ámbar/verde fosforescente)
        "ACCENT_ELEMENT": (107, 142, 35),  # Verde oliva
        "SWEEP": (152, 251, 152),          # Verde pálido para el barrido
        "TARGET_BASE": (188, 238, 104),
        "TARGET_HOVER": (255, 100, 100),   # Rojo para hover (podría ser un ámbar más brillante también)
        "COMPASS_ROSE": (188, 238, 104),
        "DATA_PANEL_BG": (30, 60, 30),     # Un verde aún más oscuro para el panel
        "DATA_PANEL_BORDER": (107, 142, 35),
        "BUTTON_BG": (30, 60, 30),
        "BUTTON_BORDER": (107, 142, 35),
        "CURSOR_CROSS": (188, 238, 104),
        "RANGE_RINGS": (107, 142, 35),     # Verde oliva para los anillos
        "SHIP_TRACK": (152, 251, 152),     # Verde pálido para la estela
        "CENTER_ICON": (188, 238, 104),
        "MENU_ITEM_HIGHLIGHT": DEFAULT_VERDE_CLARO, # Mantener este por ahora o definir uno específico
        "MENU_ITEM_RED_HIGHLIGHT": DEFAULT_ROJO, # Mantener este por ahora o definir uno específico
    },
    2: { # Verde Claro
        "BACKGROUND": (144, 238, 144),     # Verde claro (LightGreen)
        "PRIMARY_TEXT": (0, 100, 0),       # Verde oscuro para texto
        "ACCENT_ELEMENT": (34, 139, 34),   # Verde bosque
        "SWEEP": (60, 179, 113),           # Verde mar medio
        "TARGET_BASE": (0, 100, 0),
        "TARGET_HOVER": (255, 0, 0),       # Rojo estándar para hover
        "COMPASS_ROSE": (0, 100, 0),
        "DATA_PANEL_BG": (224, 255, 224),  # Verde muy pálido (Honeydew)
        "DATA_PANEL_BORDER": (34, 139, 34),
        "BUTTON_BG": (224, 255, 224),
        "BUTTON_BORDER": (34, 139, 34),
        "CURSOR_CROSS": (0, 100, 0),
        "RANGE_RINGS": (34, 139, 34),      # Verde bosque para los anillos
        "SHIP_TRACK": (60, 179, 113),      # Verde mar medio para la estela
        "CENTER_ICON": (0, 100, 0),
        "MENU_ITEM_HIGHLIGHT": DEFAULT_VERDE_CLARO,
        "MENU_ITEM_RED_HIGHLIGHT": DEFAULT_ROJO,
    },
    3: { # Azul (Actual - Default)
        "BACKGROUND": DEFAULT_AZUL,
        "PRIMARY_TEXT": DEFAULT_BLANCO,
        "ACCENT_ELEMENT": DEFAULT_VERDE,
        "SWEEP": DEFAULT_CELESTE,
        "TARGET_BASE": DEFAULT_BLANCO,
        "TARGET_HOVER": DEFAULT_ROJO,
        "COMPASS_ROSE": DEFAULT_BLANCO,
        "DATA_PANEL_BG": DEFAULT_NEGRO,
        "DATA_PANEL_BORDER": DEFAULT_VERDE,
        "BUTTON_BG": DEFAULT_NEGRO,
        "BUTTON_BORDER": DEFAULT_VERDE,
        "CURSOR_CROSS": DEFAULT_BLANCO,
        "RANGE_RINGS": DEFAULT_GRIS_MUY_CLARO,
        "SHIP_TRACK": DEFAULT_GRIS_MUY_CLARO,
        "CENTER_ICON": DEFAULT_BLANCO,
        "MENU_ITEM_HIGHLIGHT": DEFAULT_VERDE_CLARO,
        "MENU_ITEM_RED_HIGHLIGHT": DEFAULT_ROJO,
    },
    4: { # Azul Claro
        "BACKGROUND": (173, 216, 230),     # Celeste (LightBlue)
        "PRIMARY_TEXT": (0, 0, 128),       # Azul marino para texto
        "ACCENT_ELEMENT": (70, 130, 180),  # Azul acero
        "SWEEP": (135, 206, 250),          # Azul cielo claro
        "TARGET_BASE": (0, 0, 128),
        "TARGET_HOVER": (255, 0, 0),       # Rojo estándar
        "COMPASS_ROSE": (0, 0, 128),
        "DATA_PANEL_BG": (220, 240, 255),  # Azul muy pálido
        "DATA_PANEL_BORDER": (70, 130, 180),
        "BUTTON_BG": (220, 240, 255),
        "BUTTON_BORDER": (70, 130, 180),
        "CURSOR_CROSS": (0, 0, 128),
        "RANGE_RINGS": (70, 130, 180),     # Azul acero para los anillos
        "SHIP_TRACK": (135, 206, 250),     # Azul cielo claro para la estela
        "CENTER_ICON": (0, 0, 128),
        "MENU_ITEM_HIGHLIGHT": DEFAULT_VERDE_CLARO, # Podrían ser azules también
        "MENU_ITEM_RED_HIGHLIGHT": DEFAULT_ROJO,
    }
}

# Variable global para el esquema de color activo
# Se inicializa con el esquema por defecto (3)
# El valor de menu_options_values["color_menu"] (1-4) será el índice.
active_color_scheme_idx = 3 # Default
current_colors = color_schemes[active_color_scheme_idx]

# Actualizar las variables de color globales para que apunten a las del esquema actual
# Esto es para minimizar cambios extensos en el código existente que usa NEGRO, BLANCO, etc.
# En su lugar, estas variables globales ahora obtendrán su valor del current_colors.
# Sin embargo, es mejor reemplazar los usos directos de NEGRO, BLANCO, etc., con current_colors["KEY"].
# Por ahora, mantenemos algunas globales para compatibilidad, pero idealmente se refactorizarían.

NEGRO = DEFAULT_NEGRO # Usado para el fondo de botones de popup y texto en algunos casos.
BLANCO = DEFAULT_BLANCO # Usado ampliamente para texto.
VERDE = DEFAULT_VERDE   # Usado para bordes de panel, etc.
ROJO = DEFAULT_ROJO     # Para hover de targets, etc.
AZUL = DEFAULT_AZUL     # Para fondo principal.
CELESTE = DEFAULT_CELESTE # Para barrido.
VERDE_CLARO = DEFAULT_VERDE_CLARO # Para selectores cuadrados de menú.
GRIS_MEDIO = DEFAULT_GRIS_MEDIO # Para fondo de popups.
GRIS_MUY_CLARO = DEFAULT_GRIS_MUY_CLARO # Para anillos de rango.


PI = 3.141592653

# Establecemos la altura y largo de la pantalla
# Intentar obtener el tamaño del escritorio para una ventana grande inicial,
# o usar un tamaño predeterminado si falla.
try:
    display_info = pygame.display.Info()
    initial_width = display_info.current_w
    initial_height = display_info.current_h
    # Opcional: Usar un porcentaje del tamaño del escritorio o un máximo
    # initial_width = int(display_info.current_w * 0.9)
    # initial_height = int(display_info.current_h * 0.9)
except pygame.error:
    initial_width = 1024 # Fallback
    initial_height = 768  # Fallback

dimensiones = [initial_width, initial_height]
# Usar pygame.RESIZABLE para permitir que el usuario cambie el tamaño de la ventana
screen = pygame.display.set_mode(dimensiones, pygame.RESIZABLE)
pygame.display.set_caption("SIMULADOR SONAR")
pantalla = screen # Restaurar la asignación de pantalla para que sea la misma superficie que screen

# Define Range Presets and Current Range State Variable (Moved up)
RANGE_PRESETS_METERS = [100, 150, 200, 250, 300, 350, 400, 450, 500, 600, 800, 1000, 1200, 1600]
RANGE_PRESETS_BRAZAS = [50, 75, 100, 125, 150, 175, 200, 225, 250, 300, 400, 500, 600, 800]

current_unit = "METERS"
range_presets_map = {
    "METERS": RANGE_PRESETS_METERS,
    "BRAZAS": RANGE_PRESETS_BRAZAS
}
range_display_suffix_map = { # Units removed from R value displays
    "METERS": "",
    "BRAZAS": ""
}
sonar_rose_unit_text_map = {
    "METERS": "(m)",
    "BRAZAS": "(fm)"
}

# Initialize current_range_index (Moved up)
try:
    default_range_value_meters = 300
    current_range_index = RANGE_PRESETS_METERS.index(default_range_value_meters)
except ValueError:
    current_range_index = 4

# --- Puerto Pop-up Auto-Close Timer Variables ---
puerto_popup_auto_close_start_time = None
PUERTO_POPUP_AUTO_CLOSE_DELAY = 2000 # milliseconds (2 seconds)
# ---

# Font initialization
font = pygame.font.Font(None, 24) # Using a default system font
font_large = pygame.font.SysFont("segoeuisymbol", 30)
font_very_large = pygame.font.Font(None, 48)
font_data_medium = pygame.font.Font(None, 36)
font_size_50 = pygame.font.Font(None, 50)
font_size_54 = pygame.font.Font(None, 54)
font_menu_item = pygame.font.Font(None, 20) # New font for menu items

# --- Button Definitions & Initial Surfaces (Now correctly placed after font and maps) ---
# These surfaces will be re-rendered if colors change
button_unidades_text_surface = None # Will be rendered in main loop or when colors change
button_unidades_rect = pygame.Rect(0,0,0,0) # Placeholder, will be defined in main loop
active_sonar_rose_unit_surface = None # Will be rendered in main loop or when colors change
show_unidades_popup = False # For controlling popup visibility

# Pop-up UI elements (placeholders and pre-rendered text)
# These surfaces will be re-rendered if colors change
popup_main_rect = pygame.Rect(0,0,0,0) # For Unidades popup
popup_metros_text_surface = None # font.render("METROS", True, BLANCO, NEGRO)
popup_metros_option_rect = pygame.Rect(0,0,0,0)
popup_brazas_text_surface = None # font.render("BRAZAS", True, BLANCO, NEGRO)
popup_brazas_option_rect = pygame.Rect(0,0,0,0)

# --- "Puerto" (COM Port Selection) Pop-up State and UI Variables ---
show_puerto_popup = False
selected_com_port_in_popup = None # Stores the device name string
selected_baud_rate_in_popup = 9600 # Default, will store chosen baud rate
show_com_port_dropdown = False
show_baud_rate_dropdown = False
available_com_ports_list = [] # Will be populated by get_available_com_ports()
available_baud_rates_list = [4800, 9600, 19200, 38400, 57600, 115200]
puerto_popup_message = "" # For status/error messages in the port pop-up

# "Puerto" button (main screen)
# These surfaces will be re-rendered if colors change
button_puerto_text_surface = None # font.render("PUERTO", True, BLANCO, NEGRO)
button_puerto_rect = pygame.Rect(0,0,0,0) # Positioned in main loop

# "Menú" button (main screen)
button_menu_text_surface = None # font.render("MENÚ", True, BLANCO, NEGRO)
button_menu_rect = pygame.Rect(0,0,0,0) # Positioned in main loop, likely top-right

# "Puerto" Pop-up elements (rects will be defined dynamically when pop-up is shown)
puerto_popup_main_rect = pygame.Rect(0,0,0,0)
puerto_popup_select_port_rect = pygame.Rect(0,0,0,0) # Clickable area to show port list
puerto_popup_select_baud_rect = pygame.Rect(0,0,0,0) # Clickable area to show baud list
puerto_popup_apply_button_rect = pygame.Rect(0,0,0,0)
puerto_popup_cancel_button_rect = pygame.Rect(0,0,0,0)
puerto_popup_port_list_item_rects = [] # For clickable port options
puerto_popup_baud_list_item_rects = [] # For clickable baud options

# Pre-rendered text for "Puerto" pop-up (can be done once, text color will adapt)
# These surfaces will be re-rendered if colors change
puerto_popup_title_surf = None # font_large.render("Configurar Puerto", True, BLANCO)
puerto_popup_com_label_surf = None # font.render("Puerto COM:", True, BLANCO)
puerto_popup_baud_label_surf = None # font.render("Baudios:", True, BLANCO)
puerto_popup_apply_text_surf = None # font.render("Aplicar", True, BLANCO, NEGRO) # Button text
puerto_popup_cancel_text_surf = None # font.render("Cancelar", True, BLANCO, NEGRO) # Button text
# --- End "Puerto" Pop-up State and UI Variables ---

# --- Main Menu Pop-up State and Option Variables ---
show_main_menu_popup = False
# global_menu_panel_rect = None # Initialized before main loop
interactive_menu_item_rects = {} # Will store rects of interactive elements in the menu

# Default values for menu options
menu_options_values = {
    "potencia_tx": 5,           # 1-10
    "long_impulso": 5,          # 1-10
    "ciclo_tx": 5,              # 1-10
    "tvg_proximo": 5,           # 1-10
    "tvg_lejano": 5,            # 1-10
    "cag": 5,                   # 1-10
    "cag_2": 5,                 # 1-10 (2° CAG)
    "limitar_ruido": 5,         # 1-10
    "curva_color": 1,           # 1-4 (Square Red)
    "respuesta_color": 1,       # 1-4 (Square Red)
    "anular_color": 0,          # 0 (Placeholder, may become toggle or specific values later)
    "promedio_eco": 1,          # 1-3 (Dropdown)
    "rechazo_interf": 1,        # 1-3 (Dropdown)
    "angulo_haz_hor": "ANCHO",  # "ANCHO", "ESTRECHO" (Square Red)
    "angulo_haz_ver": "ANCHO",  # "ANCHO", "ESTRECHO" (Square Red)
    "color_menu": 1,            # 1-4 (Square Green Light) - Key for "COLOR" option
    # BORRAR MARCAS items are action buttons, not value states here
    "nivel_alarma": 9,          # 1-10 (Dropdown)
    "explor_auto": "ON",        # "ON", "OFF" (Square Green Light)
    "sector_explor": "±10°",    # Dropdown with specific strings
    "inclin_auto": "ON",        # "ON", "OFF" (Square Green Light)
    "angulo_inclin": "±2-10°",  # Dropdown with specific strings
    "transmision": "ON",        # "ON", "OFF" (Square Green Light)
    "volumen_audio": 10,        # 1-10 (Dropdown)
    # ASIGNAR AJUSTE and ASIGNAR MENU are placeholders/actions, not direct value states here
    # "nivel_alarma": 9,          # REMOVED
    # "explor_auto": "ON",        # REMOVED
    # "sector_explor": "±10°",    # REMOVED
    # "inclin_auto": "ON",        # REMOVED
    # "angulo_inclin": "±2-10°",  # REMOVED
}

# States for individual dropdowns within the main menu
menu_dropdown_states = {
    "potencia_tx": False,
    "long_impulso": False,
    "ciclo_tx": False,
    "tvg_proximo": False,
    "tvg_lejano": False,
    "cag": False,
    "cag_2": False,
    "limitar_ruido": False,
    "anular_color": False,      # Will be a dropdown 1-10 as per original full list
    "promedio_eco": False,
    "rechazo_interf": False,
    # "nivel_alarma": False,    # REMOVED
    # "sector_explor": False,   # REMOVED
    # "angulo_inclin": False,   # REMOVED
    "volumen_audio": False
}
# --- End Main Menu Pop-up State and Option Variables ---

# --- NMEA Transition State ---
prev_serial_port_available = False # Tracks if NMEA was available in the previous frame for conversion logic

# --- Automatic COM Port Reconnection State ---
last_reconnect_attempt_time = 0
RECONNECT_INTERVAL_MS = 5000 # 5 seconds

# --- Ship Track Variables ---
ship_track_points = [] 
MAX_TRACK_DISTANCE_METERS = 5 * 1852  # 5 Nautical Miles in meters
TRACK_POINT_INTERVAL_MS = 1000      # Add a track point every 1 second
last_track_point_add_time = 0       # Timestamp of the last added track point
    # COLOR_TRACK = GRIS_MUY_CLARO        # Color for the track line - Will use current_colors["SHIP_TRACK"]
# --- End Ship Track Variables ---

# --- Sonar Sweep Variables ---
SPEED_OF_SOUND_MPS = 1500  # Speed of sound in water in meters/second
current_sweep_radius_pixels = 0
# --- End Sonar Sweep Variables ---

# --- Audio Volume ---
current_volume = 1.0 # Max volume (0.0 to 1.0 for pygame)
# --- End Audio Volume ---

# --- Load Sonar Sound ---
try:
    # Construct path to sound file relative to the script's location
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sound_file_path = os.path.join(script_dir, "eco.mp3")
    sonar_ping_sound = pygame.mixer.Sound(sound_file_path)
    sonar_ping_sound.set_volume(current_volume) # Establecer volumen inicial
except pygame.error as e:
    print(f"Advertencia: No se pudo cargar el archivo de sonido '{sound_file_path if 'sound_file_path' in locals() else 'eco.mp3'}': {e}")
    print("El programa continuará sin sonido de barrido.")
    sonar_ping_sound = None
# --- End Load Sonar Sound ---

# Text string definitions (already added in previous step, kept for completeness)
texto_latitud = "LAT / LON "
texto_longitud = "VELOC DEL BARCO" # This is "Ship Speed" label
texto_velocidad = "RUMBO DEL BARCO" # This is "Ship Course" label
texto_dato_nuevo_1 = "HORA / FECHA"
# texto_dato_nuevo_2 = "RATE OF TURN" # ROT text definition removed
texto_dato_nuevo_3 = "PITCH / ROLL"


# Usado para gestionar cuán rápido se actualiza la pantalla
reloj = pygame.time.Clock()

# Display box dimensions
# display_box_1_dims = [620, 10, 280, 340]  # Combined VEL/RUMBO/LATLON - Replaced
# combined_data_box_dims = [620, 360, 280, 350] # Increased height from 270 to 350 - Replaced
# unified_data_box_dims = [620, 10, 280, 700] # Old unified box for 910x635 screen
# Para 1024x768, el sonar principal podría ser más grande.
# Si el sonar es de 700x700 (antes 600x600), y empieza en (10,10)
# El panel de datos podría empezar en x = 700 + 10 + 10 = 720.
# Ancho del panel de datos: 1024 - 720 - 10 (margen derecho) = 294.
# Altura del panel de datos: 768 - 10 (margen superior) - 10 (margen inferior) = 748.
unified_data_box_dims = [720, 10, 294, 748] # New unified box for 1024x768

# Tilt Control Variables
current_tilt_angle = 0
MIN_TILT = 0
MAX_TILT = 55
show_tilt_temporarily = False
tilt_display_timer = 0
TILT_DISPLAY_DURATION_FRAMES = 60 # Approx 1 second at 60 FPS

# Temporary Range Display Variables
show_range_temporarily = False
range_display_timer = 0
RANGE_DISPLAY_DURATION_FRAMES = 60 # Approx 1 second at 60 FPS

# Gain Control Variables
current_gain = 0.0
MIN_GAIN = 0.0
MAX_GAIN = 10.0
GAIN_INCREMENT = 0.5 # Changed from 0.1 to 0.5
show_gain_temporarily = False
gain_display_timer = 0
GAIN_DISPLAY_DURATION_FRAMES = 60 # Approx 1 second at 60 FPS

# --- Target Management System ---
target_markers = [] # List to store target dictionaries
TARGET_TYPE_RHOMBUS = "rhombus"
TARGET_TYPE_X = "x"
# All markers will be initially white. Hovering will make them red.
# COLOR_TARGET_BASE = BLANCO # Base color for all markers - Will use current_colors["TARGET_BASE"]
# COLOR_TARGET_HOVER = ROJO   # Color when hovered or selected - Will use current_colors["TARGET_HOVER"]

# --- UI State Dictionary ---
ui_state = {
    "show_plus_cursor": False,
    "mouse_cursor_pos": (0, 0),
    "cursor_H_proj_display": "---", # Horizontal Projection
    "cursor_S_range_display": "---", # Slant Range to cursor point
    "cursor_Depth_display": "---",  # Depth at cursor point
    "cursor_bearing_display": "---", # Bearing of cursor from center
    # For target calculations display - Initialized to "---"
    "target_dist_t1_t2": "---",
    "target_dist_center_t2": "---",
    "target_depth_t2": "---",
    "target_speed_t1_t2": "---",
    "target_course_t1_t2": "---",
    "hovered_marker_index": None # To store the index of the marker being hovered
}

# --- Key Event Handling Function ---
def handle_key_events(event_key, circle_center_x_param, circle_center_y_param, display_radius_pixels_param, s_max_current_range_param):
    global current_range_index, current_tilt_angle, show_tilt_temporarily, tilt_display_timer, \
           show_range_temporarily, range_display_timer, \
           current_gain, show_gain_temporarily, gain_display_timer, target_markers, \
           current_ship_lat_deg, current_ship_lon_deg, current_ship_heading, current_unit, \
           current_volume, menu_options_values, sonar_ping_sound
    # Constants like RANGE_PRESETS_METERS, MIN_TILT, MAX_TILT, etc., are accessible globally

    if event_key == pygame.K_y:
        current_range_index += 1
        # Ensure index is valid for the current unit's presets
        current_range_index = min(current_range_index, len(range_presets_map[current_unit]) - 1)
        show_range_temporarily = True
        range_display_timer = RANGE_DISPLAY_DURATION_FRAMES
    elif event_key == pygame.K_h:
        current_range_index -= 1
        current_range_index = max(0, current_range_index)
        show_range_temporarily = True
        range_display_timer = RANGE_DISPLAY_DURATION_FRAMES
    elif event_key == pygame.K_u: # Tilt Up (meaning angle decreases towards 0)
        current_tilt_angle = max(current_tilt_angle - 1, MIN_TILT)
        show_tilt_temporarily = True
        tilt_display_timer = TILT_DISPLAY_DURATION_FRAMES
    elif event_key == pygame.K_j: # Tilt Down (meaning angle increases towards 55)
        current_tilt_angle = min(current_tilt_angle + 1, MAX_TILT)
        show_tilt_temporarily = True
        tilt_display_timer = TILT_DISPLAY_DURATION_FRAMES
    elif event_key == pygame.K_o:  # Gain Up
        current_gain = min(round(current_gain + GAIN_INCREMENT, 1), MAX_GAIN)
        show_gain_temporarily = True
        gain_display_timer = GAIN_DISPLAY_DURATION_FRAMES
    elif event_key == pygame.K_l:  # Gain Down
        current_gain = max(round(current_gain - GAIN_INCREMENT, 1), MIN_GAIN)
        show_gain_temporarily = True
        gain_display_timer = GAIN_DISPLAY_DURATION_FRAMES
    elif event_key == pygame.K_f:
        if ui_state["show_plus_cursor"]: # Only add marker if cursor is active in sonar circle
            mouse_cursor_x, mouse_cursor_y = ui_state["mouse_cursor_pos"]
            current_time = pygame.time.get_ticks()

            new_marker = {
                'mode': None, # Will be 'geo' or 'screen'
                'geo_pos': None, # For 'geo' mode
                'initial_screen_pos': None, # For 'screen' mode: (abs_x, abs_y) at creation, less critical now
                'initial_distance_meters': None, # For 'screen' mode: conceptual distance from center, in METERS
                'original_angle_rad': None, # For 'screen' mode: standard math angle (atan2(dy,dx)) from center
                'screen_bearing_rad': None, # For 'screen' mode: visual angle from screen up (atan2(dx,-dy)) for NMEA conversion
                'type': TARGET_TYPE_RHOMBUS,
                # 'color': COLOR_TARGET_BASE, # Color will be determined by current_colors and hover state
                'timestamp': current_time,
                'current_screen_pos': None,
                'is_hovered': False
            }

            if current_ship_lat_deg is not None and current_ship_lon_deg is not None:
                new_marker['mode'] = 'geo'
                # Calculate and store geo_pos for GEO marker
                pixel_dist_from_center = math.sqrt((mouse_cursor_x - circle_center_x_param)**2 + (mouse_cursor_y - circle_center_y_param)**2)
                # distance_m_from_ship is the true slant distance in meters
                distance_m_from_ship = (pixel_dist_from_center / display_radius_pixels_param) * s_max_current_range_param if display_radius_pixels_param > 0 else 0
                if current_unit == "BRAZAS": # s_max_current_range_param is in brazas, convert to meters for geo calc
                    distance_m_from_ship *= 1.8288
                
                dx = mouse_cursor_x - circle_center_x_param
                dy = mouse_cursor_y - circle_center_y_param # Pygame y is inverted for visual angle
                visual_angle_rad = math.atan2(dx, -dy) # atan2(x,y) for bearing from North axis
                visual_bearing_deg = (math.degrees(visual_angle_rad) + 360) % 360
                true_bearing_deg = (visual_bearing_deg + current_ship_heading) % 360

                start_point = Point(latitude=current_ship_lat_deg, longitude=current_ship_lon_deg)
                if distance_m_from_ship < 0: distance_m_from_ship = 0 # Should not happen with sqrt
                destination = geodesic(meters=distance_m_from_ship).destination(point=start_point, bearing=true_bearing_deg)
                new_marker['geo_pos'] = {'lat': destination.latitude, 'lon': destination.longitude}
            else:
                # SCREEN marker: No NMEA data
                new_marker['mode'] = 'screen'
                new_marker['initial_screen_pos'] = (mouse_cursor_x, mouse_cursor_y) # Absolute screen pixels
                new_marker['initial_S_max'] = s_max_current_range_param # Sonar range (units) at creation
                new_marker['initial_display_radius_pixels'] = display_radius_pixels_param # Sonar radius (pixels) at creation
                
                # Calculate conceptual distance in current display units
                pixel_dist_from_center = math.sqrt((mouse_cursor_x - circle_center_x_param)**2 + (mouse_cursor_y - circle_center_y_param)**2)
                if display_radius_pixels_param > 0:
                    conceptual_distance_in_current_units = (pixel_dist_from_center / display_radius_pixels_param) * s_max_current_range_param
                else:
                    conceptual_distance_in_current_units = 0
                
                # Convert to meters and store
                if current_unit == "BRAZAS": # current_unit is global, reflects unit at marker creation
                    new_marker['initial_distance_meters'] = conceptual_distance_in_current_units * 1.8288
                else: # Assumed METERS
                    new_marker['initial_distance_meters'] = conceptual_distance_in_current_units
                
                # Calculate and store the original angle from center
                dx_orig = mouse_cursor_x - circle_center_x_param
                dy_orig = mouse_cursor_y - circle_center_y_param
                new_marker['original_angle_rad'] = math.atan2(dy_orig, dx_orig) # Standard math angle (0 rad = screen right)
                # Calculate and store screen bearing (0 rad = screen up, positive clockwise)
                new_marker['screen_bearing_rad'] = math.atan2(dx_orig, -dy_orig) 

                new_marker['geo_pos'] = None
            
            # Common logic for adding marker and updating previous ones
            # Type (shape) update remains.
            if len(target_markers) > 0: # If there's at least one marker already
                # The one just about to become "previous" (currently the last one)
                # should retain its shape (Rhombus) but will be COLOR_TARGET_BASE.
                # Its color will only change if hovered.
                if len(target_markers) >= 2: # If there are at least two markers already
                    # The one that was T1 (target_markers[-2]) now becomes T0 (the oldest displayable X)
                    older_marker = target_markers[-2]
                    older_marker['type'] = TARGET_TYPE_X
                    # Its color is already COLOR_TARGET_BASE or will be set by hover.
            
            target_markers.append(new_marker)
    
    elif event_key == pygame.K_d:
        if ui_state['hovered_marker_index'] is not None and \
           ui_state['hovered_marker_index'] < len(target_markers):
            target_markers.pop(ui_state['hovered_marker_index'])
            ui_state['hovered_marker_index'] = None # Clear hover after delete

            # Re-evaluate T0, T1, T2 shapes after deletion
            # The newest one (if any) is T2 (Rhombus)
            # The one before that (if any) is T1 (Rhombus)
            # The one before T1 (if any) is T0 (X)
            # All others remain X if they were already X.
            
            if len(target_markers) >= 1:
                target_markers[-1]['type'] = TARGET_TYPE_RHOMBUS
            if len(target_markers) >= 2:
                 target_markers[-2]['type'] = TARGET_TYPE_RHOMBUS # This was T1, should remain Rhombus or become the new T2
            
            # All markers older than the (potential new) T1 should be X
            # This means if we have 3 or more markers, the third from the end (and older) become X.
            # If we have target_markers[idx], T2 is at -1, T1 at -2. T0 (first X) is at -3.
            if len(target_markers) >=3:
                 for i in range(len(target_markers) - 2): # All except last two
                      target_markers[i]['type'] = TARGET_TYPE_X
            
            # Ensure correct types if only 0, 1 or 2 markers remain
            if len(target_markers) == 1: # Only one marker, it's a rhombus
                target_markers[0]['type'] = TARGET_TYPE_RHOMBUS
            elif len(target_markers) == 2: # Two markers, both rhombus
                target_markers[0]['type'] = TARGET_TYPE_RHOMBUS
                target_markers[1]['type'] = TARGET_TYPE_RHOMBUS


# --- End Key Event Handling Function ---

def draw_rhombus(surface, color, center_x, center_y, size):
    """Draws a rhombus shape."""
    half_size = size // 2
    points = [
        (center_x, center_y - half_size),  # Top point
        (center_x + half_size, center_y),  # Right point
        (center_x, center_y + half_size),  # Bottom point
        (center_x - half_size, center_y)   # Left point
    ]
    pygame.draw.polygon(surface, color, points, 2) # Added width=2 for border only

def draw_x_mark(surface, color, center_x, center_y, size):
    """Draws an 'X' shape."""
    half_size = size // 2
    line_thickness = 2 # Same as cursor lines
    # Line 1: Top-left to Bottom-right
    pygame.draw.line(surface, color, 
                     (center_x - half_size, center_y - half_size), 
                     (center_x + half_size, center_y + half_size), line_thickness)
    # Line 2: Top-right to Bottom-left
    pygame.draw.line(surface, color, 
                     (center_x + half_size, center_y - half_size), 
                     (center_x - half_size, center_y + half_size), line_thickness)

# --- Ship Track Logic ---
def update_ship_track():
    global last_track_point_add_time, ship_track_points, current_ship_lat_deg, current_ship_lon_deg

    if current_ship_lat_deg is None or current_ship_lon_deg is None:
        if len(ship_track_points) > 0: # Clear track if we lose position
            # print("Lost ship position, clearing track.") # Debug
            ship_track_points.clear()
        return

    current_time = pygame.time.get_ticks()
    if current_time - last_track_point_add_time >= TRACK_POINT_INTERVAL_MS:
        new_point = {'lat': current_ship_lat_deg, 'lon': current_ship_lon_deg}
        ship_track_points.append(new_point)
        last_track_point_add_time = current_time
        # print(f"Added track point: {new_point}, Total points: {len(ship_track_points)}") # Debug

        # Manage track length
        current_track_length_meters = 0
        points_to_keep = []
        if len(ship_track_points) >= 2:
            # Iterate in reverse to sum distances from newest to oldest
            # and keep points that fall within the MAX_TRACK_DISTANCE_METERS
            temp_points_in_range = [ship_track_points[-1]] # Always keep the last point
            for i in range(len(ship_track_points) - 2, -1, -1):
                p1 = Point(latitude=ship_track_points[i+1]['lat'], longitude=ship_track_points[i+1]['lon'])
                p2 = Point(latitude=ship_track_points[i]['lat'], longitude=ship_track_points[i]['lon'])
                segment_distance = geodesic(p1, p2).meters
                if current_track_length_meters + segment_distance <= MAX_TRACK_DISTANCE_METERS:
                    current_track_length_meters += segment_distance
                    temp_points_in_range.insert(0, ship_track_points[i]) # Insert at beginning to maintain order
                else:
                    break # Stop adding older points
            ship_track_points = temp_points_in_range
            # print(f"Track managed. Points: {len(ship_track_points)}, Length: {current_track_length_meters:.2f}m") # Debug

# --- End Ship Track Logic ---

# --- Track Drawing Logic ---
def get_geo_line_circle_intersection(p1_geo, p2_geo, center_geo, radius_m):
    """
    Finds the intersection of a line segment (p1_geo, p2_geo) with a circle 
    (center_geo, radius_m). Returns the intersection point closest to p1_geo
    if p1_geo is inside and p2_geo is outside, or None.
    This is a simplified approach and might need a more robust geometric solution
    for perfect accuracy, especially with geographic coordinates.
    For now, it interpolates linearly in geographic space, which is an approximation.
    """
    # Simplified: Check distances and interpolate.
    # A more accurate method would project to a 2D plane or use vector math.
    dist_p1_to_center = geodesic(center_geo, p1_geo).meters
    dist_p2_to_center = geodesic(center_geo, p2_geo).meters

    # If p1 is outside and p2 is inside, swap them for consistent logic
    if dist_p1_to_center > radius_m and dist_p2_to_center <= radius_m:
        p1_geo, p2_geo = p2_geo, p1_geo
        # dist_p1_to_center, dist_p2_to_center = dist_p2_to_center, dist_p1_to_center # Not needed after swap

    # Recalculate distances after potential swap
    dist_p1_to_center = geodesic(center_geo, p1_geo).meters
    dist_p2_to_center = geodesic(center_geo, p2_geo).meters

    if dist_p1_to_center <= radius_m and dist_p2_to_center > radius_m:
        # p1 is inside, p2 is outside. Find intersection.
        # Linear interpolation factor
        # Total "distance" of the segment part that is outside vs inside relative to radius
        # This is a simplification.
        if dist_p2_to_center - dist_p1_to_center == 0: # Avoid division by zero if points are identical
            return p1_geo # Or handle as an error/edge case

        # t is the fraction of the segment from p1 to p2 where the intersection occurs
        t = (radius_m - dist_p1_to_center) / (dist_p2_to_center - dist_p1_to_center)
        
        # Clamp t to [0, 1] in case of floating point inaccuracies or edge cases
        t = max(0, min(1, t))

        # Interpolate coordinates
        intersect_lat = p1_geo.latitude + t * (p2_geo.latitude - p1_geo.latitude)
        intersect_lon = p1_geo.longitude + t * (p2_geo.longitude - p1_geo.longitude)
        return Point(latitude=intersect_lat, longitude=intersect_lon)
    return None

# --- Screen Coordinate Line-Circle Intersection for Target Markers ---
def get_screen_line_circle_intersection(p1_screen, p2_screen, circle_center_x, circle_center_y, circle_radius_pixels):
    # p1 is assumed to be inside or on the circle, p2 is used for direction and assumed to be outside.
    # Calculates intersection of the ray p1->p2 with the circle.
    # Uses normalized direction vector for better numerical stability.

    x1, y1 = p1_screen
    x2, y2 = p2_screen # p2 is used for direction; can be far outside
    cx, cy = circle_center_x, circle_center_y
    r = circle_radius_pixels

    # Translate points so circle is at origin for easier math
    x1_trans, y1_trans = x1 - cx, y1 - cy
    
    # Direction vector from p1 to p2
    dx = x2 - x1
    dy = y2 - y1

    len_d = math.sqrt(dx*dx + dy*dy)
    if len_d < 1e-9: # p1 and p2 are essentially the same point
        # If p1 is on the circle, it's the "intersection"
        if abs((x1_trans**2 + y1_trans**2) - r**2) < 1e-6:
            return p1_screen
        return None # Otherwise, no line segment to intersect from p1

    udx, udy = dx/len_d, dy/len_d # Unit direction vector from p1 towards p2

    # Line is P_trans = P1_trans + k * U_dir, where k is distance along unit vector
    # Substitute into circle equation: (x1_trans + k*udx)^2 + (y1_trans + k*udy)^2 = r^2
    # Expands to: k^2*(udx^2+udy^2) + k*(2*x1_trans*udx + 2*y1_trans*udy) + (x1_trans^2+y1_trans^2-r^2) = 0
    # Since (udx^2+udy^2) = 1 (it's a unit vector):
    # k^2 + k*(2 * (x1_trans*udx + y1_trans*udy)) + (x1_trans^2+y1_trans^2-r^2) = 0
    
    # Quadratic equation Ak^2 + Bk_coeff*k + Ck_coeff = 0 for k
    A_k = 1.0
    B_k_coeff = 2 * (x1_trans * udx + y1_trans * udy)
    C_k_coeff = (x1_trans**2 + y1_trans**2) - r**2

    discriminant_k = B_k_coeff**2 - 4*A_k*C_k_coeff

    if discriminant_k < -1e-9: # Allow for small negative due to precision
        return None # No real intersection (line misses the circle)
    
    # Clamp if slightly negative due to floating point math
    if discriminant_k < 0: discriminant_k = 0 

    sqrt_discriminant_k = math.sqrt(discriminant_k)
    
    # Two solutions for k (distance from p1_trans along unit vector)
    k1 = (-B_k_coeff + sqrt_discriminant_k) / (2*A_k)
    k2 = (-B_k_coeff - sqrt_discriminant_k) / (2*A_k)

    # We want the smallest non-negative k.
    # This represents the first intersection point *forward* from p1 along the ray towards p2.
    # If p1 is inside the circle (C_k_coeff < 0), one k will be positive (exit) and one negative.
    # If p1 is on the circle (C_k_coeff ~ 0), one k is ~0, other is positive/negative.
    # If p1 is outside (C_k_coeff > 0, though this function assumes p1 is inside/on),
    #   both k could be positive (entry and exit) or complex (miss).
    
    intersect_k = -1.0 # Initialize with an invalid value

    if k1 >= -1e-9: # k1 is a potential forward intersection (allowing for precision around 0)
        intersect_k = k1
    
    if k2 >= -1e-9: # k2 is also a potential forward intersection
        if intersect_k == -1.0 or k2 < intersect_k: # If k1 was not valid or k2 is smaller
            intersect_k = k2
            
    if intersect_k == -1.0: # No valid non-negative k found
        return None

    # The intersection point must be on the segment defined by p1 and extending towards p2,
    # up to the conceptual location of p2.
    # intersect_k is the distance from p1. If intersect_k > len_d, it means p2 was *inside*
    # the circle, and the intersection is beyond p2 (on the ray).
    # This function is for when p1 is inside and we are looking for the exit towards an outside p2.
    # So, intersect_k should generally be <= len_d if p2 defines the actual segment end.
    # However, p2_screen is often a "far point" for direction, so len_d can be huge.
    # The crucial part is that intersect_k is the distance from p1 to the circle edge.

    # Calculate the intersection point in original screen coordinates
    final_intersect_x = x1 + intersect_k * udx
    final_intersect_y = y1 + intersect_k * udy
    
    return (int(round(final_intersect_x)), int(round(final_intersect_y)))
# --- End Screen Coordinate Line-Circle Intersection ---

# --- Menu Drawing Helper Function ---
def draw_single_dropdown_option(surface, option_key_name, label_text, current_opt_value,
                                is_dropdown_open, y_pos, parent_panel_rect, font_obj,
                                color_map, item_row_height, dd_item_h,
                                item_list_or_range): # Changed from num_dd_items
    """
    Draws a single dropdown option within the main menu.
    item_list_or_range: Can be a list of strings or a range object (e.g., range(1, 11)).
    Returns a dictionary of clickable rects: {'value_box': Rect, 'items': [Rects], 'item_values': [actual_item_values]}.
    """
    drawn_rects = {'value_box': None, 'items': [], 'item_values': []} # Added item_values
    padding_x = 10 # Internal padding for items in the panel

    # Determine the actual list of items to display (strings)
    if isinstance(item_list_or_range, range):
        display_items = [str(i) for i in item_list_or_range]
        actual_item_values = list(item_list_or_range)
    elif isinstance(item_list_or_range, list):
        display_items = [str(item) for item in item_list_or_range] # Ensure all are strings for rendering
        actual_item_values = item_list_or_range # Keep original values for logic
    else: # Fallback for unexpected type
        display_items = []
        actual_item_values = []

    # 1. Draw Label
    label_surface = font_obj.render(label_text, True, color_map["BLANCO"])
    label_position_rect = label_surface.get_rect(topleft=(parent_panel_rect.left + padding_x, y_pos))
    surface.blit(label_surface, label_position_rect)

    # 2. Draw Value Display Box (clickable to toggle dropdown)
    value_box_width = 50  # Standard width for the value box
    value_box_height = item_row_height
    value_box_rect_obj = pygame.Rect(
        label_position_rect.right + 5, # 5px space after label
        y_pos,
        value_box_width,
        value_box_height
    )
    pygame.draw.rect(surface, color_map["NEGRO"], value_box_rect_obj)
    pygame.draw.rect(surface, color_map["BLANCO"], value_box_rect_obj, 1) # Border
    
    value_text_surface = font_obj.render(str(current_opt_value), True, color_map["BLANCO"])
    value_text_position_rect = value_text_surface.get_rect(center=value_box_rect_obj.center)
    surface.blit(value_text_surface, value_text_position_rect)
    drawn_rects['value_box'] = value_box_rect_obj

    # 3. Draw Dropdown List (if open)
    dropdown_total_height = 0
    if is_dropdown_open:
        dd_box_x = value_box_rect_obj.left
        dd_box_y = value_box_rect_obj.bottom + 2 # 2px space below value box
        dd_box_width = value_box_rect_obj.width
        
        # Background for the entire dropdown list
        dropdown_total_height = len(display_items) * dd_item_h # Use length of actual items
        if dropdown_total_height > 0 : # Ensure there's something to draw
            pygame.draw.rect(surface, color_map["NEGRO"], (dd_box_x, dd_box_y, dd_box_width, dropdown_total_height))
            pygame.draw.rect(surface, color_map["BLANCO"], (dd_box_x, dd_box_y, dd_box_width, dropdown_total_height), 1) # Border

            current_item_y = dd_box_y
            for item_text_to_display in display_items:
                item_surface = font_obj.render(item_text_to_display, True, color_map["BLANCO"])
                
                # Center text vertically within the item slot
                text_y_offset = (dd_item_h - item_surface.get_height()) // 2
                item_pos_rect = item_surface.get_rect(left=dd_box_x + 5, top=current_item_y + text_y_offset)
                surface.blit(item_surface, item_pos_rect)
                
                # Store rect for the whole clickable area of the item
                clickable_item_area_rect = pygame.Rect(dd_box_x, current_item_y, dd_box_width, dd_item_h)
                drawn_rects['items'].append(clickable_item_area_rect)
                current_item_y += dd_item_h
            
            drawn_rects['item_values'] = actual_item_values # Store the actual values corresponding to items

    # Return the height occupied by this item (including open dropdown if any)
    # This is useful for the calling code to manage y_offset for next items.
    # Height is the row height + dropdown height if open.
    occupied_height = item_row_height
    if is_dropdown_open and dropdown_total_height > 0: # Add height only if dropdown was actually drawn
        occupied_height += (2 + dropdown_total_height) # 2 is spacing

    return drawn_rects, occupied_height
# --- End Menu Drawing Helper Function ---

# --- Menu Click Handling Helper Function ---
def handle_menu_dropdown_click(event_pos, option_key, base_value_box_rect, 
                               item_rect_list, item_value_list): # Removed num_items, added item_value_list
    """
    Handles clicks for a single dropdown option.
    Updates menu_options_values and menu_dropdown_states.
    Returns True if a click related to this dropdown was processed, False otherwise.
    """
    global menu_options_values, menu_dropdown_states, current_colors, active_color_scheme_idx

    # 1. Check click on the value_box (to open/close dropdown)
    if base_value_box_rect and base_value_box_rect.collidepoint(event_pos):
        menu_dropdown_states[option_key] = not menu_dropdown_states[option_key]
        # Close all other dropdowns
        for key in menu_dropdown_states:
            if key != option_key:
                menu_dropdown_states[key] = False
        return True # Click was processed

    # 2. Check click on an item in an open dropdown
    if menu_dropdown_states[option_key] and item_rect_list and item_value_list:
        for i, item_rect in enumerate(item_rect_list):
            if item_rect.collidepoint(event_pos):
                if i < len(item_value_list): # Ensure index is valid
                    new_value = item_value_list[i]
                    menu_options_values[option_key] = new_value
                    menu_dropdown_states[option_key] = False # Close this dropdown

                    if option_key == "volumen_audio":
                        global current_volume, sonar_ping_sound
                        current_volume = new_value / 10.0
                        if sonar_ping_sound:
                            sonar_ping_sound.set_volume(current_volume)
                        print(f"Volumen cambiado desde menú a: {new_value}") # Debug
                    return True # Click was processed
        
        # If click was in the dropdown's general area but not on an item,
        # and also not on the value_box itself (that's handled above).
        # This could mean a click on the scrollbar area if implemented, or just empty space.
        # For now, if it's not an item, we don't consume the click here, letting the main
        # menu's "click outside to close" logic handle it if necessary.
        # However, we need to ensure the click was at least within the logical dropdown bounds
        # to prevent closing if the click was on another UI element that happens to be behind an open dropdown.
        # This check can be complex. A simpler approach for now: if it's not an item, it's not handled by *this* part.
        
    return False # No click related to this specific dropdown was processed by this function call
# --- End Menu Click Handling Helper Function ---

# --- Square Selector Drawing Helper ---
def draw_square_selector_option(surface, option_key_name, label_text, current_opt_value,
                                square_item_labels, # List of labels for each square (e.g., ["1", "2"] or ["ANCHO", "ESTRECHO"])
                                y_pos, parent_panel_rect, font_obj,
                                color_map, item_row_height, square_size, 
                                highlight_color_rgb, text_color_rgb=None): # text_color_rgb is optional for text on squares
    """
    Draws a square/box selection type menu option.
    Returns a list of clickable Rects for the squares and the total height occupied.
    """
    drawn_item_rects = []
    padding_x = 10
    square_spacing = 5
    if text_color_rgb is None: # Default text color on squares
        text_color_rgb = color_map["BLANCO"]

    # 1. Draw Label
    label_surface = font_obj.render(label_text, True, color_map["BLANCO"])
    label_position_rect = label_surface.get_rect(topleft=(parent_panel_rect.left + padding_x, y_pos))
    surface.blit(label_surface, label_position_rect)

    current_x_offset = label_position_rect.right + 10 # Start drawing squares after label + padding

    for i, item_label_text in enumerate(square_item_labels):
        square_rect = pygame.Rect(current_x_offset, y_pos, square_size, item_row_height) # Use item_row_height for square height
        
        # Calculate width for this specific square based on its label if it's ANCHO/ESTRECHO type
        current_square_width = square_size
        if item_label_text.upper() in ["ANCHO", "ESTRECHO"]:
            text_w = font_obj.size(item_label_text)[0]
            current_square_width = text_w + 10 # 5px padding on each side of text
        
        square_rect = pygame.Rect(current_x_offset, y_pos, current_square_width, item_row_height)
        
        # Determine if selected
        is_selected = False
        # Ensure consistent type comparison (e.g. if current_opt_value is int like 1, and item_label_text is "1")
        if isinstance(current_opt_value, str):
            is_selected = (current_opt_value.upper() == item_label_text.upper())
        elif isinstance(current_opt_value, int):
            try:
                is_selected = (int(item_label_text) == current_opt_value)
            except ValueError:
                 pass # Not a numeric label, won't match int current_opt_value

        text_draw_color = text_color_rgb # Default text color
        if is_selected:
            pygame.draw.rect(surface, highlight_color_rgb, square_rect) # Fill selected
            pygame.draw.rect(surface, color_map["BLANCO"], square_rect, 1) # White border for filled
            # Potentially change text color for contrast on filled background
            # Example: if highlight is dark, use white text. If light, use black.
            # This needs more sophisticated color contrast logic or fixed contrasting colors.
            # For ROJO and VERDE_CLARO, BLANCO text should generally be fine.
            text_draw_color = color_map["BLANCO"] 
        else:
            pygame.draw.rect(surface, color_map["NEGRO"], square_rect) # Standard background
            pygame.draw.rect(surface, color_map["BLANCO"], square_rect, 1) # Standard border

        # Draw text label inside square
        item_text_surf = font_obj.render(str(item_label_text), True, text_draw_color)
        item_text_rect = item_text_surf.get_rect(center=square_rect.center)
        surface.blit(item_text_surf, item_text_rect)
        
        drawn_item_rects.append(square_rect)
        current_x_offset += square_size + square_spacing

    return drawn_item_rects, item_row_height # Occupied height is just the row height
# --- End Square Selector Drawing Helper ---

# --- Square Selector Click Handling Helper ---
def handle_square_selector_click(event_pos, option_key_name, square_rects_list, option_values_list):
    """
    Handles clicks for a square selector option.
    Updates menu_options_values.
    Returns True if a click was processed, False otherwise.
    """
    global menu_options_values, current_colors, active_color_scheme_idx
    for i, rect in enumerate(square_rects_list):
        if rect.collidepoint(event_pos):
            if i < len(option_values_list): # Make sure index is valid
                new_value = option_values_list[i]
                menu_options_values[option_key_name] = new_value

                if option_key_name == "color_menu":
                    active_color_scheme_idx = int(new_value) # color_menu stores 1,2,3,4 as string or int
                    current_colors = color_schemes[active_color_scheme_idx]
                    # Need to trigger re-render of text surfaces that depend on these colors
                    # This will be handled by making their surface variables None and re-rendering in main loop,
                    # or by a dedicated function to update all themed surfaces.
                    # For now, the main loop will pick up the new current_colors.
                    print(f"Color scheme changed to: {active_color_scheme_idx}")

                return True # Click processed
    return False
# --- End Square Selector Click Handling Helper ---

# --- Action Button Drawing Helper ---
def draw_action_button(surface, text, button_rect, font_obj, color_map, border_color=None, text_color=None):
    """
    Draws a simple action button.
    Returns the rect of the button.
    """
    if border_color is None:
        border_color = color_map["BLANCO"]
    if text_color is None:
        text_color = color_map["BLANCO"]

    pygame.draw.rect(surface, color_map["NEGRO"], button_rect)
    pygame.draw.rect(surface, border_color, button_rect, 1)
    
    text_surf = font_obj.render(text, True, text_color)
    text_rect = text_surf.get_rect(center=button_rect.center)
    surface.blit(text_surf, text_rect)
    return button_rect # Return the rect for click handling
# --- End Action Button Drawing Helper ---

# --- Action Button Click Handling Helper ---
def handle_action_button_click(event_pos, button_rect_to_check, action_key_name):
    """
    Handles a click for a simple action button.
    For now, just prints a message.
    Returns True if the click was on this button, False otherwise.
    """
    if button_rect_to_check and button_rect_to_check.collidepoint(event_pos):
        print(f"Action button '{action_key_name}' (text: {button_rect_to_check.width}) clicked!") # Placeholder action
        # In a real scenario, this would trigger some game logic or state change
        # For "BORRAR MARCAS", it might be:
        # if action_key_name == "borrar_derrota": clear_derrota_action()
        # etc.
        return True
    return False
# --- End Action Button Click Handling Helper ---


def draw_ship_track(surface, track_points_geo, ship_lat, ship_lon, ship_hdg_deg,
                    cc_x, cc_y, disp_radius_px, s_max_on_disp, current_disp_unit, track_color):
    if ship_lat is None or ship_lon is None:
        return

    s_max_meters_on_display = s_max_on_disp
    if current_disp_unit == "BRAZAS":
        s_max_meters_on_display *= 1.8288

    current_ship_geo = Point(latitude=ship_lat, longitude=ship_lon)
    
    # The track to draw will be composed of multiple segments (lines)
    # Each segment is a list of screen points
    segments_to_draw = []
    current_segment_screen_points = []

    # Combine current ship position with historic track points for segment processing
    # The track runs from oldest historic point to current ship position
    full_track_geo_points = track_points_geo + [{'lat': ship_lat, 'lon': ship_lon}]

    if len(full_track_geo_points) < 2:
        return # Need at least two points to form a line

    for i in range(len(full_track_geo_points) - 1):
        p1_geo_data = full_track_geo_points[i]
        p2_geo_data = full_track_geo_points[i+1]
        
        p1_geo = Point(latitude=p1_geo_data['lat'], longitude=p1_geo_data['lon'])
        p2_geo = Point(latitude=p2_geo_data['lat'], longitude=p2_geo_data['lon'])

        dist_p1_from_ship_m = geodesic(current_ship_geo, p1_geo).meters
        dist_p2_from_ship_m = geodesic(current_ship_geo, p2_geo).meters

        p1_is_visible = dist_p1_from_ship_m <= s_max_meters_on_display
        p2_is_visible = dist_p2_from_ship_m <= s_max_meters_on_display

        # Function to convert a geo point to screen coordinates
        def geo_to_screen(geo_pt, ref_ship_geo, ref_ship_hdg_deg, 
                          center_x, center_y, display_radius_pixels, 
                          max_dist_meters_for_display_edge):
            dist_m = geodesic(ref_ship_geo, geo_pt).meters
            if dist_m > max_dist_meters_for_display_edge + 1: # Add a small buffer
                 return None # Clearly outside

            # Calculate true bearing from ship to geo_pt
            delta_lon_rad = math.radians(geo_pt.longitude - ref_ship_geo.longitude)
            lat1_rad = math.radians(ref_ship_geo.latitude)
            lat2_rad = math.radians(geo_pt.latitude)
            y_brg = math.sin(delta_lon_rad) * math.cos(lat2_rad)
            x_brg = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                    math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)
            true_bearing_rad = math.atan2(y_brg, x_brg)
            true_bearing_deg = (math.degrees(true_bearing_rad) + 360) % 360
            
            relative_bearing_deg = (true_bearing_deg - ref_ship_hdg_deg + 360) % 360
            
            pixel_dist = (dist_m / max_dist_meters_for_display_edge) * display_radius_pixels \
                         if max_dist_meters_for_display_edge > 0 else 0
            
            angle_rad_draw = math.radians(relative_bearing_deg)
            
            scr_x = center_x + pixel_dist * math.sin(angle_rad_draw)
            scr_y = center_y - pixel_dist * math.cos(angle_rad_draw) # Pygame Y is inverted
            return (int(round(scr_x)), int(round(scr_y)))

        p1_screen = geo_to_screen(p1_geo, current_ship_geo, ship_hdg_deg, cc_x, cc_y, disp_radius_px, s_max_meters_on_display)
        p2_screen = geo_to_screen(p2_geo, current_ship_geo, ship_hdg_deg, cc_x, cc_y, disp_radius_px, s_max_meters_on_display)

        if p1_is_visible and p2_is_visible:
            # Both points are visible, add p1_screen to current segment if not already there
            if not current_segment_screen_points or current_segment_screen_points[-1] != p1_screen:
                if p1_screen: current_segment_screen_points.append(p1_screen)
            if p2_screen: current_segment_screen_points.append(p2_screen)
        
        elif p1_is_visible and not p2_is_visible:
            # p1 visible, p2 not. Find intersection.
            intersection_geo = get_geo_line_circle_intersection(p1_geo, p2_geo, current_ship_geo, s_max_meters_on_display)
            if intersection_geo:
                intersection_screen = geo_to_screen(intersection_geo, current_ship_geo, ship_hdg_deg, cc_x, cc_y, disp_radius_px, s_max_meters_on_display)
                if not current_segment_screen_points or current_segment_screen_points[-1] != p1_screen:
                     if p1_screen: current_segment_screen_points.append(p1_screen)
                if intersection_screen: current_segment_screen_points.append(intersection_screen)
            
            if current_segment_screen_points:
                segments_to_draw.append(list(current_segment_screen_points)) # End current segment
                current_segment_screen_points.clear()

        elif not p1_is_visible and p2_is_visible:
            # p1 not visible, p2 visible. Find intersection.
            intersection_geo = get_geo_line_circle_intersection(p2_geo, p1_geo, current_ship_geo, s_max_meters_on_display) # Note order swap for get_geo_line_circle_intersection
            if intersection_geo:
                intersection_screen = geo_to_screen(intersection_geo, current_ship_geo, ship_hdg_deg, cc_x, cc_y, disp_radius_px, s_max_meters_on_display)
                # Start new segment with intersection point
                if intersection_screen: current_segment_screen_points.append(intersection_screen)
            if p2_screen: current_segment_screen_points.append(p2_screen)
            # This segment will continue or be added in the next iteration if p2 was the last point.

        elif not p1_is_visible and not p2_is_visible:
            # Both points outside. If there was an ongoing segment, end it.
            if current_segment_screen_points:
                segments_to_draw.append(list(current_segment_screen_points))
                current_segment_screen_points.clear()
            # No part of this segment is drawn.

    # Add any remaining segment
    if current_segment_screen_points:
        segments_to_draw.append(list(current_segment_screen_points))

    # Draw all collected segments
    for segment in segments_to_draw:
        # Remove duplicate consecutive points that might arise from intersections
        unique_points_in_segment = []
        if segment:
            unique_points_in_segment.append(segment[0])
            for k in range(1, len(segment)):
                if segment[k] != segment[k-1]:
                    unique_points_in_segment.append(segment[k])
        
        if len(unique_points_in_segment) >= 2:
            pygame.draw.lines(surface, track_color, False, unique_points_in_segment, 1)

# --- End Track Drawing Logic ---

# --- Calculation Logic for Targets ---
def calculate_target_data(targets, current_tilt_angle_deg, S_max_range, display_radius_px, 
                          current_unit_str, circle_center_coords, current_ship_hdg_deg):
    """
    Calculates various data points based on the last two target markers.
    Updates ui_state with the results.
    """
    # Reset display values first
    ui_state['target_dist_t1_t2'] = "---"
    ui_state['target_dist_center_t2'] = "---"
    ui_state['target_depth_t2'] = "---"
    ui_state['target_speed_t1_t2'] = "---"
    ui_state['target_course_t1_t2'] = "---"

    if len(targets) < 1:
        return # Not enough targets for any calculation

    # --- Calculations for the last target (T2) ---
    t2 = targets[-1]
    cc_x, cc_y = circle_center_coords # Unpack for screen calculations

    # --- T2: Distance from Center & Depth ---
    if t2['mode'] == 'geo' and t2['geo_pos'] and current_ship_lat_deg is not None and current_ship_lon_deg is not None:
        ship_point = Point(latitude=current_ship_lat_deg, longitude=current_ship_lon_deg)
        t2_geo_point = Point(latitude=t2['geo_pos']['lat'], longitude=t2['geo_pos']['lon'])
        dist_meters_ship_to_t2 = geodesic(ship_point, t2_geo_point).meters
        
        s_range_t2_display_units = dist_meters_ship_to_t2
        if current_unit_str == "BRAZAS": s_range_t2_display_units /= 1.8288
        ui_state['target_dist_center_t2'] = f"{int(round(s_range_t2_display_units))}" # No unit suffix

        tilt_rad_t2 = math.radians(current_tilt_angle_deg)
        depth_t2 = s_range_t2_display_units * math.sin(tilt_rad_t2)
        ui_state['target_depth_t2'] = f"{int(round(depth_t2))}" # No unit suffix

    elif t2['mode'] == 'screen' and t2['initial_screen_pos']:
        t2_scr_x, t2_scr_y = t2['initial_screen_pos']
        pixel_dist_center_to_t2_scr = math.sqrt((t2_scr_x - cc_x)**2 + (t2_scr_y - cc_y)**2)
        s_range_t2_scr = (pixel_dist_center_to_t2_scr / display_radius_px) * S_max_range if display_radius_px > 0 else 0
        # S_max_range is already in display units, so s_range_t2_scr is too.
        ui_state['target_dist_center_t2'] = f"{int(round(s_range_t2_scr))}" # No unit suffix

        tilt_rad_t2_scr = math.radians(current_tilt_angle_deg)
        depth_t2_scr = s_range_t2_scr * math.sin(tilt_rad_t2_scr)
        ui_state['target_depth_t2'] = f"{int(round(depth_t2_scr))}" # No unit suffix
    else: # Not enough info for T2 calcs
        ui_state['target_dist_center_t2'] = "---"
        ui_state['target_depth_t2'] = "---"


    if len(targets) < 2: # Not enough for T1-T2 calculations
        return 

    # --- Calculations involving the last two targets (T1 and T2) ---
    t1 = targets[-2] 

    if t1['mode'] == 'geo' and t2['mode'] == 'geo' and t1['geo_pos'] and t2['geo_pos']:
        t1_geo_point = Point(latitude=t1['geo_pos']['lat'], longitude=t1['geo_pos']['lon'])
        t2_geo_point = Point(latitude=t2['geo_pos']['lat'], longitude=t2['geo_pos']['lon']) # Already defined if T2 was geo

        dist_meters_t1_t2 = geodesic(t1_geo_point, t2_geo_point).meters
        dist_t1_t2_display_units = dist_meters_t1_t2
        if current_unit_str == "BRAZAS": dist_t1_t2_display_units /= 1.8288
        ui_state['target_dist_t1_t2'] = f"{int(round(dist_t1_t2_display_units))}" # No unit suffix

        time_diff_ms = t2['timestamp'] - t1['timestamp']
        if time_diff_ms > 0:
            speed_m_per_s_t1_t2 = dist_meters_t1_t2 / (time_diff_ms / 1000.0)
            ui_state['target_speed_t1_t2'] = f"{(speed_m_per_s_t1_t2 * 1.94384):.1f} kn"
        else:
            ui_state['target_speed_t1_t2'] = "N/A"

        delta_lon_t1_t2 = math.radians(t2_geo_point.longitude - t1_geo_point.longitude)
        lat1_rad_t1 = math.radians(t1_geo_point.latitude)
        lat2_rad_t2 = math.radians(t2_geo_point.latitude)
        y_brg = math.sin(delta_lon_t1_t2) * math.cos(lat2_rad_t2)
        x_brg = math.cos(lat1_rad_t1) * math.sin(lat2_rad_t2) - math.sin(lat1_rad_t1) * math.cos(lat2_rad_t2) * math.cos(delta_lon_t1_t2)
        true_bearing_t1_to_t2_deg = (math.degrees(math.atan2(y_brg, x_brg)) + 360) % 360
        ui_state['target_course_t1_t2'] = f"{int(round(true_bearing_t1_to_t2_deg))}°"

    elif t1['mode'] == 'screen' and t2['mode'] == 'screen' and t1['initial_screen_pos'] and t2['initial_screen_pos']:
        t1_scr_x, t1_scr_y = t1['initial_screen_pos']
        t2_scr_x, t2_scr_y = t2['initial_screen_pos']

        pixel_dist_t1_t2_scr = math.sqrt((t2_scr_x - t1_scr_x)**2 + (t2_scr_y - t1_scr_y)**2)
        dist_t1_t2_scr = (pixel_dist_t1_t2_scr / display_radius_px) * S_max_range if display_radius_px > 0 else 0
        # S_max_range is already in display units
        ui_state['target_dist_t1_t2'] = f"{int(round(dist_t1_t2_scr))}" # No unit suffix
        
        time_diff_ms_scr = t2['timestamp'] - t1['timestamp']
        if time_diff_ms_scr > 0:
            speed_disp_units_per_sec = dist_t1_t2_scr / (time_diff_ms_scr / 1000.0)
            speed_m_per_s = speed_disp_units_per_sec
            if current_unit_str == "BRAZAS": speed_m_per_s *= 1.8288 # convert fm/s to m/s
            ui_state['target_speed_t1_t2'] = f"{(speed_m_per_s * 1.94384):.1f} kn"
        else:
            ui_state['target_speed_t1_t2'] = "N/A"

        dx_scr = t2_scr_x - t1_scr_x
        dy_scr = t2_scr_y - t1_scr_y
        if dx_scr == 0 and dy_scr == 0: course_scr_deg = 0
        else: course_scr_deg = (math.degrees(math.atan2(dx_scr, -dy_scr)) + 360) % 360 # Relative to screen North
        # Convert to "absolute" by adding ship heading, if available
        abs_course_scr_deg = (course_scr_deg + current_ship_hdg_deg) % 360 if current_ship_hdg_deg is not None else course_scr_deg
        ui_state['target_course_t1_t2'] = f"{int(round(abs_course_scr_deg))}°"
        
    else: # Mixed modes or missing data for T1/T2 calcs
        ui_state['target_dist_t1_t2'] = "---"
        ui_state['target_speed_t1_t2'] = "---"
        ui_state['target_course_t1_t2'] = "---"

# --- Real-time Screen Position Calculation for Markers ---
def update_marker_screen_positions(targets, ship_lat, ship_lon, ship_hdg_deg, 
                                   cc_x, cc_y, disp_radius_px, s_max_disp_range, current_disp_unit):
    """
    Updates the 'current_screen_pos' for each geographically-locked target marker
    based on the current ship position and heading.
    """
    # Updates the 'current_screen_pos' for each target marker.
    # Handles 'geo' markers based on ship position and 'screen' markers based on display changes.

    ship_point = None
    if ship_lat is not None and ship_lon is not None:
        ship_point = Point(latitude=ship_lat, longitude=ship_lon)

    for marker in targets:
        if marker['mode'] == 'geo':
            if ship_point and marker['geo_pos']:
                target_point = Point(latitude=marker['geo_pos']['lat'], longitude=marker['geo_pos']['lon'])
                dist_meters_to_target = geodesic(ship_point, target_point).meters
                
                s_max_meters_on_display = s_max_disp_range # s_max_disp_range is already in display units
                if current_disp_unit == "BRAZAS": # if display is brazas, convert s_max_disp_range (brazas) to meters
                    s_max_meters_on_display *= 1.8288

                if dist_meters_to_target > s_max_meters_on_display:
                    marker['current_screen_pos'] = None
                    continue

                # Calculate true bearing from ship to target
                delta_lon = math.radians(target_point.longitude - ship_point.longitude)
                lat1_rad = math.radians(ship_point.latitude)
                lat2_rad = math.radians(target_point.latitude)
                y_brg = math.sin(delta_lon) * math.cos(lat2_rad)
                x_brg = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon)
                true_bearing_to_target_rad = math.atan2(y_brg, x_brg)
                true_bearing_to_target_deg = (math.degrees(true_bearing_to_target_rad) + 360) % 360
                
                # Relative bearing for drawing on screen (0 deg is UP)
                relative_bearing_deg = (true_bearing_to_target_deg - ship_hdg_deg + 360) % 360
                angle_rad_for_draw = math.radians(relative_bearing_deg - 90) # Adjust for Pygame: 0=East, -90=North/Up
                
                # Pixel distance from center
                # s_max_disp_range is in current display units (m or fm)
                # dist_meters_to_target needs to be converted to current display units if they differ.
                dist_units_to_target = dist_meters_to_target
                if current_disp_unit == "BRAZAS":
                    dist_units_to_target /= 1.8288 # Convert meters to brazas for proportional calculation

                pixel_dist_from_center = (dist_units_to_target / s_max_disp_range) * disp_radius_px if s_max_disp_range > 0 else 0
                
                screen_x_rel_center = pixel_dist_from_center * math.cos(angle_rad_for_draw)
                screen_y_rel_center = pixel_dist_from_center * math.sin(angle_rad_for_draw) 
                
                marker_screen_x = cc_x + screen_x_rel_center
                marker_screen_y = cc_y + screen_y_rel_center # Pygame's Y is already handled by angle adjustment
                marker['current_screen_pos'] = (int(round(marker_screen_x)), int(round(marker_screen_y)))
            else:
                marker['current_screen_pos'] = None # No ship or geo position for geo marker

        elif marker['mode'] == 'screen':
            # 'Screen' mode markers always rescale with the current sonar display settings,
            # regardless of NMEA status, as they are not tied to a geographic point.
            if marker.get('initial_distance_meters') is not None and \
               marker.get('original_angle_rad') is not None and \
               s_max_disp_range > 0 and disp_radius_px > 0: # s_max_disp_range is in current_disp_unit
                
                marker_dist_m = marker['initial_distance_meters']
                angle_rad_initial = marker['original_angle_rad'] # This is atan2(dy,dx)

                # Convert current display's max range to meters for comparison
                s_max_meters_on_display = s_max_disp_range
                if current_disp_unit == "BRAZAS": # current_disp_unit is argument to this function
                    s_max_meters_on_display *= 1.8288

                if marker_dist_m <= s_max_meters_on_display:
                    # Both marker_dist_m and s_max_meters_on_display are in meters.
                    # Ratio is correct for scaling against pixel radius.
                    new_pixel_dist = (marker_dist_m / s_max_meters_on_display) * disp_radius_px \
                                     if s_max_meters_on_display > 0 else 0 # Avoid division by zero if s_max_meters is 0
                    
                    # original_angle_rad (atan2(dy,dx)) means:
                    # x = cos(angle), y = sin(angle) for standard math axes
                    # Pygame screen: x_screen = center_x + R*cos(angle), y_screen = center_y + R*sin(angle)
                    new_x = cc_x + new_pixel_dist * math.cos(angle_rad_initial)
                    new_y = cc_y + new_pixel_dist * math.sin(angle_rad_initial)
                    marker['current_screen_pos'] = (int(round(new_x)), int(round(new_y)))
                else:
                    marker['current_screen_pos'] = None # Marker's conceptual distance in meters is outside current display range in meters
            
            # Fallback conditions if rescaling data is incomplete.
            elif marker.get('initial_screen_pos') is not None:
                marker['current_screen_pos'] = marker['initial_screen_pos']
            
            else:
                # No information to position the marker.
                marker['current_screen_pos'] = None
        else:
            marker['current_screen_pos'] = None # Unknown mode

def get_line_circle_intersection(p1, p2, circle_center, circle_radius):
    """
    Calculates the intersection point of a line segment starting at p1 and directed towards p2,
    with a circle. Assumes p1 is inside or on the circle, and p2 is used to define direction
    (p2 can be outside the circle).
    Args:
        p1: Tuple (x, y) for the start point of the line segment (must be inside/on circle).
        p2: Tuple (x, y) for the end point defining the line's direction.
        circle_center: Tuple (cx, cy) for the center of the circle.
        circle_radius: Float for the radius of the circle.
    Returns:
        Tuple (x, y) of the intersection point on the circle's edge, or
        None if p1 and p2 are the same, or if no valid forward intersection is found.
    """
    x1, y1 = p1
    x2, y2 = p2
    cx, cy = circle_center

    # Vector from p1 to p2
    dx = x2 - x1
    dy = y2 - y1

    if abs(dx) < 1e-9 and abs(dy) < 1e-9: # p1 and p2 are very close or the same
        return None 

    # Quadratic equation parameters for t: a_quad*t^2 + b_quad*t + c_quad = 0
    # where the intersection point (x,y) = (x1 + t*dx, y1 + t*dy)
    
    # Vector from circle center to p1
    p1_cx = x1 - cx
    p1_cy = y1 - cy
    
    a_quad = dx**2 + dy**2
    if a_quad < 1e-9: # Effectively dx and dy are zero, should have been caught by the check above.
        return None

    b_quad = 2 * (dx * p1_cx + dy * p1_cy)
    c_quad = p1_cx**2 + p1_cy**2 - circle_radius**2

    discriminant = b_quad**2 - 4 * a_quad * c_quad

    if discriminant < 0:
        # No real intersection.
        return None 

    sqrt_discriminant = math.sqrt(discriminant)
    
    # Solve for t.
    t_sol1 = (-b_quad - sqrt_discriminant) / (2 * a_quad)
    t_sol2 = (-b_quad + sqrt_discriminant) / (2 * a_quad)
    
    # We are looking for the smallest t >= 0 (or very close to 0, using a small epsilon for float precision).
    # This represents the first intersection point along the ray from p1 towards p2.
    
    intersect_t = float('inf')
    valid_t_candidate_found = False

    # Check t_sol1
    if t_sol1 >= -1e-9: # Allow for t being extremely close to 0 from the negative side due to precision
        intersect_t = t_sol1
        valid_t_candidate_found = True

    # Check t_sol2
    if t_sol2 >= -1e-9:
        if not valid_t_candidate_found or t_sol2 < intersect_t:
            intersect_t = t_sol2
        valid_t_candidate_found = True
            
    if not valid_t_candidate_found:
        # Both intersection points are "behind" p1 (both t substantially < 0).
        return None 
        
    # If p1 is inside the circle (c_quad < 0), one t must be positive and one negative.
    # The logic above (smallest t >= -epsilon) will correctly pick the positive t.
    # If p1 is on the circle (c_quad ~ 0), one t is ~0. If the line points outwards,
    # the other t is positive. Smallest t >= -epsilon still works.
    # If p1 is outside the circle (c_quad > 0), both t could be positive if line crosses.
    # This function assumes p1 is inside or on the circle for clipping an outgoing line.

    # If the calculated intersection_t would result in a point that is essentially p1,
    # and p2 is distinct from p1, it might mean p1 is on the edge and the line is tangent
    # or points inward. In such cases, for clipping an outgoing line, there's no "further" intersection.
    if intersect_t < 1e-9 : # If t is effectively zero
        # Check if p1 is already on or very near the circle boundary
        dist_p1_sq = p1_cx**2 + p1_cy**2
        if abs(dist_p1_sq - circle_radius**2) < 1e-6:
            # p1 is on the circle. If the line points outwards, t should be positive.
            # If intersect_t is still ~0, it means the line doesn't extend further out along the circle.
            # This can happen if p2 is also on the circle or inside.
            # If p2 is outside, and t=0 is the only non-negative solution, line is tangent at p1 or points inward.
             dist_p2_from_center_sq = (x2 - cx)**2 + (y2 - cy)**2 # Added definition for dist_p2_from_center_sq
             if dist_p2_from_center_sq > circle_radius**2 + 1e-6 : # if p2 is outside
                 # if b_quad (which is 2 * dot_product(vec_p1_p2, vec_center_p1)) is >= 0, line points inwards or tangent
                 # We need line pointing outwards for a valid t > 0 (or t=0 if p1 is the exit)
                 if b_quad >= 0 and not (t_sol1 > 1e-9 or t_sol2 > 1e-9) : # No other positive t
                     return None # No outward intersection from p1 if it's on the edge and line doesn't go out.
            # else p2 is inside or on, p1 is the "intersection"
            # return p1 # or None, depending on desired behavior for line of zero length
    
    # If after all checks, intersect_t is still effectively zero or negative,
    # it means there's no valid *forward* intersection from p1.
    if intersect_t < 1e-9 and not (abs(dx) < 1e-9 and abs(dy) < 1e-9) : # Check again, ensure it's meaningfully positive
         # This condition can be tricky. If p1 is on the edge, t=0 is a valid intersection.
         # The drawing code should handle a line from p1 to p1.
         # Let's assume smallest t >= -epsilon is what we want.
         pass


    intersect_x = x1 + intersect_t * dx
    intersect_y = y1 + intersect_t * dy
    
    return (int(round(intersect_x)), int(round(intersect_y)))

def draw_center_icon(surface, center_x, center_y, total_height, color):
    """Draws a vertical rectangle topped with a triangle at the given center."""
    icon_width = total_height / 3 
    rect_height = total_height * (2/3)
    tri_height = total_height * (1/3) 
    half_width = icon_width / 2
    icon_total_half_height = total_height / 2

    # Y-coordinates based on centering the whole icon
    tri_peak_y = center_y - icon_total_half_height
    rect_top_y = tri_peak_y + tri_height
    rect_bottom_y = rect_top_y + rect_height

    # Rectangle points (outline)
    rect_points = [
        (int(center_x - half_width), int(rect_top_y)),
        (int(center_x + half_width), int(rect_top_y)),
        (int(center_x + half_width), int(rect_bottom_y)),
        (int(center_x - half_width), int(rect_bottom_y))
    ]
    pygame.draw.polygon(surface, color, rect_points, 5) # Thickness 5

    # Triangle points (outline)
    tri_points = [
        (int(center_x), int(tri_peak_y)),
        (int(center_x - half_width), int(rect_top_y)),
        (int(center_x + half_width), int(rect_top_y))
    ]
    pygame.draw.polygon(surface, color, tri_points, 5) # Thickness 5


def draw_dotted_ellipse(surface, color, rect, dot_radius, spacing_angle):
    """Draws a dotted ellipse."""
    center_x = rect[0] + rect[2] / 2
    center_y = rect[1] + rect[3] / 2
    radius_x = rect[2] / 2
    radius_y = rect[3] / 2

    if radius_x <= 0 or radius_y <= 0: # Avoid math errors for zero or negative radii
        return

    for angle_deg in range(0, 360, spacing_angle):
        angle_rad = math.radians(angle_deg)
        x = center_x + radius_x * math.cos(angle_rad)
        y = center_y + radius_y * math.sin(angle_rad)
        pygame.draw.circle(surface, color, (int(x), int(y)), dot_radius)


def draw_compass_rose(surface, center_x, center_y, radius, font, color, current_heading_degrees):
    """Draws a compass rose with cardinal points and 10-degree ticks, rotated by current_heading_degrees."""
    cardinal_points = {
        0: "N",
        90: "E",
        180: "S",
        270: "W"
    }
    tick_length_short = 5
    tick_length_long = 10
    # label_offset = 20 # Original offset, used for placing labels outside
    label_offset_inside = 25 # New offset for placing labels inside

    for angle_deg in range(0, 360, 10):
        # Adjust angle for Pygame's coordinate system (0 degrees is typically East)
        # We want 0 degrees (North) to be at the top.
        # math.sin and math.cos expect radians.
        # Standard angle: 0=East, 90=North, 180=West, 270=South
        # Pygame angle for drawing: 0=Top (North), 90=Right (East), 180=Bottom (South), 270=Left (West)
        # So, we subtract 90 degrees from our desired compass angle.
        # We also subtract the current_heading_degrees to rotate the rose.
        # The true North (0 degrees on the rose) should align with the ship's current heading.
        display_angle_deg = angle_deg - current_heading_degrees
        angle_rad = math.radians(display_angle_deg - 90)

        is_cardinal = angle_deg in cardinal_points # Still check original angle_deg for labels
        current_tick_length = tick_length_long if is_cardinal else tick_length_short

        # Calculate tick start and end points
        # Tick starts from (radius - tick_length) and goes to radius
        x_outer = center_x + radius * math.cos(angle_rad)
        y_outer = center_y + radius * math.sin(angle_rad)
        x_inner = center_x + (radius - current_tick_length) * math.cos(angle_rad)
        y_inner = center_y + (radius - current_tick_length) * math.sin(angle_rad)
        pygame.draw.line(surface, color, (x_inner, y_inner), (x_outer, y_outer), 2)

        if is_cardinal:
            label_text = cardinal_points[angle_deg]
            # Use a slightly smaller font for compass labels if needed, or pass a specific one
            # For now, using the same font passed to the function.
            text_surface_compass = font.render(label_text, True, color)
            text_rect_compass = text_surface_compass.get_rect()

            # Position labels slightly inside the main circle
            # The label_offset_inside determines how far from the circle edge the text is centered.
            label_x = center_x + (radius - label_offset_inside) * math.cos(angle_rad)
            label_y = center_y + (radius - label_offset_inside) * math.sin(angle_rad)
            text_rect_compass.center = (label_x, label_y)
            surface.blit(text_surface_compass, text_rect_compass)

#Iteramos hasta que el usuario haga click sobre el botón de cerrar
hecho = False

angulo = 0

# Store rects of drawn interactive elements for click handling (main menu)
# This needs to be global or passed around if click handling is outside the main loop's direct scope
# For now, keeping it within the main loop's scope and ensuring it's cleared/updated.
# If menu state is managed outside, this might need to be part of that state.
# Declared global earlier, but ensuring it's clear it's used here.
# interactive_menu_item_rects = {} # This is now initialized inside the menu drawing block
global_menu_panel_rect = None # Initialize here, before the main loop

while not hecho:
 
    # --- Recalcular dimensiones de UI basadas en el tamaño actual de la ventana (`dimensiones`) ---
    # Primero, definir el ancho del panel de datos. Podría ser fijo o un porcentaje.
    # --- Recalcular dimensiones de UI basadas en el tamaño actual de la ventana (`dimensiones`) ---
    
    # 1. Panel de Datos (anclado a la derecha)
    data_panel_margin_right = 10
    data_panel_margin_top = 10
    data_panel_margin_bottom = 10
    
    # Ancho del panel de datos: un porcentaje de la pantalla o un valor fijo/limitado.
    # Queremos que sea lo suficientemente ancho para el contenido pero no demasiado.
    # Usaremos un valor similar al anterior (ej. 280-350px) si es posible.
    data_panel_width = min(max(int(dimensiones[0] * 0.28), 280), 320) 
    # Asegurar que el panel no sea más ancho que la pantalla menos márgenes para el sonar.
    data_panel_width = min(data_panel_width, dimensiones[0] - 200 - 30) # (min sonar width + margins)


    data_panel_height = dimensiones[1] - data_panel_margin_top - data_panel_margin_bottom
    data_panel_x = dimensiones[0] - data_panel_width - data_panel_margin_right
    data_panel_y = data_panel_margin_top
    
    # Actualizar la variable global unified_data_box_dims
    unified_data_box_dims[0] = data_panel_x
    unified_data_box_dims[1] = data_panel_y
    unified_data_box_dims[2] = data_panel_width
    unified_data_box_dims[3] = data_panel_height

    # 2. Círculo del Sonar (ocupa el espacio a la izquierda del panel de datos)
    sonar_margin_left = 10
    sonar_margin_top = 10
    sonar_margin_bottom = 10
    sonar_margin_right_to_panel = 10

    available_width_for_sonar = data_panel_x - sonar_margin_left - sonar_margin_right_to_panel
    available_height_for_sonar = dimensiones[1] - sonar_margin_top - sonar_margin_bottom
    
    sonar_diameter = min(available_width_for_sonar, available_height_for_sonar)
    sonar_diameter = max(sonar_diameter, 200) # Diámetro mínimo usable

    circle_origin_x = sonar_margin_left
    circle_origin_y = sonar_margin_top
    circle_width = sonar_diameter
    circle_height = sonar_diameter
        
    # Actualizar unified_data_box_dims globalmente si es necesario, o pasar como parámetros.
    # Por ahora, lo calcularemos aquí y lo usaremos directamente en las secciones de dibujo.
    # Si alguna función externa lo necesita, habría que considerar cómo pasarlo.
    # Para este ejemplo, asumimos que las funciones de dibujo pueden acceder a estas variables locales
    # o que `unified_data_box_dims` se actualiza aquí para ser usado más abajo.
    # La actualización de la variable global `unified_data_box_dims` ya se hizo arriba.

    circle_center_x = circle_origin_x + circle_width // 2
    circle_center_y = circle_origin_y + circle_height // 2
    display_radius_pixels = circle_width // 2

    # --- Mouse Tracking Logic ---
    mouse_x, mouse_y = pygame.mouse.get_pos()
    dist_to_center = math.sqrt((mouse_x - circle_center_x)**2 + (mouse_y - circle_center_y)**2)

    if dist_to_center <= display_radius_pixels:
        pygame.mouse.set_visible(False)
        ui_state["show_plus_cursor"] = True
        ui_state["mouse_cursor_pos"] = (mouse_x, mouse_y)

        # Calculate metric distance
        pixel_dist = dist_to_center
        # Ensure current_range_index is valid before accessing presets
        if current_range_index >= len(range_presets_map[current_unit]):
            current_range_index = len(range_presets_map[current_unit]) - 1
        S_max = range_presets_map[current_unit][current_range_index] # Max slant range for display edge
        
        if display_radius_pixels > 0:
            S_cursor = (pixel_dist / display_radius_pixels) * S_max # Slant range to cursor point ("posicion del cursor horizontal (distancia)")
            
            H_display = S_cursor # Default if no tilt (Horizontal Projection = Slant Range)
            D_display = 0.0      # Default if no tilt (Depth = 0)

            if current_tilt_angle != 0:
                tilt_rad = math.radians(current_tilt_angle)
                H_display = S_cursor * math.cos(tilt_rad) # Horizontal projection ("distancia diagonal" in user example)
                D_display = S_cursor * math.sin(tilt_rad) # Depth ("profundidad" in user example)

            # Bearing Calculation
            dx = mouse_x - circle_center_x
            dy = mouse_y - circle_center_y
            if pixel_dist == 0: # Cursor exactly at center
                bearing_deg_normalized = 0 
            else:
                angle_rad_atan2 = math.atan2(dy, dx)
                # atan2: 0 is East. Pygame y is inverted.
                # +90 to make North 0. +360 to ensure positive before final modulo.
                bearing_deg_normalized = (math.degrees(angle_rad_atan2) + 90 + 360) % 360

            ui_state["cursor_H_proj_display"] = f"{int(round(H_display))}"
            ui_state["cursor_S_range_display"] = f"{int(round(S_cursor))}" 
            ui_state["cursor_Depth_display"] = f"{int(round(D_display))}"
            ui_state["cursor_bearing_display"] = f"{int(round(bearing_deg_normalized))}"
        else:
            ui_state["cursor_H_proj_display"] = "Error"
            ui_state["cursor_S_range_display"] = "Error"
            ui_state["cursor_Depth_display"] = "Error"
            ui_state["cursor_bearing_display"] = "Error"
    else:
        pygame.mouse.set_visible(True)
        ui_state["show_plus_cursor"] = False
        ui_state["cursor_H_proj_display"] = "---"
        ui_state["cursor_S_range_display"] = "---"
        ui_state["cursor_Depth_display"] = "---"
        ui_state["cursor_bearing_display"] = "---"
    # --- End Mouse Tracking Logic ---

    # --- Calculate Target Data ---
    # Ensure current_range_index is valid before accessing presets for S_max
    if current_range_index >= len(range_presets_map[current_unit]):
        current_range_index = len(range_presets_map[current_unit]) - 1
    s_max_for_calc = range_presets_map[current_unit][current_range_index]
    
    calculate_target_data(target_markers, current_tilt_angle, s_max_for_calc, 
                          display_radius_pixels, current_unit, 
                          (circle_center_x, circle_center_y), current_ship_heading)
    # --- End Calculate Target Data ---

    # --- Update Marker Screen Positions based on Geo Pos ---
    # Ensure current_range_index is valid before accessing presets for S_max
    if current_range_index >= len(range_presets_map[current_unit]):
        current_range_index = len(range_presets_map[current_unit]) - 1
    s_max_for_update = range_presets_map[current_unit][current_range_index]

    update_marker_screen_positions(target_markers, current_ship_lat_deg, current_ship_lon_deg,
                                   current_ship_heading, circle_center_x, circle_center_y,
                                   display_radius_pixels, s_max_for_update, current_unit)
    # --- End Update Marker Screen Positions ---

    # --- Hover Logic for Target Markers ---
    ui_state['hovered_marker_index'] = None # Reset hover state each frame
    if not show_unidades_popup and not show_puerto_popup and not show_main_menu_popup: # Also check main menu
        for i, marker_data in enumerate(target_markers):
            marker_data['is_hovered'] = False # Reset individual hover state
            if marker_data['current_screen_pos']:
                mx, my = marker_data['current_screen_pos']
                # Use marker_icon_size for collision rect, already defined in drawing section
                # Make sure marker_icon_size is accessible here or use its value (18)
                marker_icon_size = 18 # Defined here for clarity
                hover_rect = pygame.Rect(mx - marker_icon_size // 2, my - marker_icon_size // 2, marker_icon_size, marker_icon_size)
                if hover_rect.collidepoint(mouse_x, mouse_y):
                    ui_state['hovered_marker_index'] = i
                    marker_data['is_hovered'] = True
                    break # Only one marker can be hovered at a time
    # --- End Hover Logic ---

    # --- Puerto Pop-up Auto-Close Logic ---
    if show_puerto_popup and puerto_popup_auto_close_start_time is not None:
        if pygame.time.get_ticks() - puerto_popup_auto_close_start_time >= PUERTO_POPUP_AUTO_CLOSE_DELAY:
            show_puerto_popup = False
            show_com_port_dropdown = False  # Ensure dropdowns are also closed
            show_baud_rate_dropdown = False
            puerto_popup_auto_close_start_time = None # Reset timer
            # puerto_popup_message = "" # Optionally clear message here or let it persist
    # --- End Puerto Pop-up Auto-Close Logic ---

    # --- Update Ship Track ---
    update_ship_track()
    # --- End Update Ship Track ---

    # --- Sonar Sweep Parameter Calculation ---
    FPS = 60 # Assuming 60 FPS from reloj.tick(60)
    # Get current max range in display units
    current_max_range_display_units = range_presets_map[current_unit][current_range_index]
    
    # Convert max range to meters
    current_max_range_meters = current_max_range_display_units
    if current_unit == "BRAZAS":
        current_max_range_meters *= 1.8288 # 1 braza = 1.8288 meters
    
    time_to_max_range_oneway_s = 0
    if SPEED_OF_SOUND_MPS > 0:
        time_to_max_range_oneway_s = current_max_range_meters / SPEED_OF_SOUND_MPS
    
    sweep_increment_ppf = 0 # Pixels per frame
    if time_to_max_range_oneway_s > 0:
        # To make the sweep twice as slow, we make it take twice as long.
        effective_time_for_sweep_s = time_to_max_range_oneway_s * 2
        # Pixels per second for the sweep to reach the edge in effective_time_for_sweep_s
        sweep_pixels_per_second = display_radius_pixels / effective_time_for_sweep_s
        sweep_increment_ppf = sweep_pixels_per_second / FPS
    
    # --- Sonar Sweep Animation Logic ---
    if menu_options_values["transmision"] == "ON":
        current_sweep_radius_pixels += sweep_increment_ppf
        if current_sweep_radius_pixels > display_radius_pixels:
            current_sweep_radius_pixels = 0 # Reset sweep
            if sonar_ping_sound: # Play sound if loaded successfully
                sonar_ping_sound.play()
    else:
        # If transmission is OFF, we can choose to reset the sweep
        # or leave it at its current position. Resetting is cleaner.
        current_sweep_radius_pixels = 0
        # Ensure sound is stopped if it was somehow playing or cued
        if sonar_ping_sound and pygame.mixer.get_busy(): # Check if any channel is busy
            sonar_ping_sound.stop() # Stop the specific sound if it's playing
            # Alternatively, stop all sounds: pygame.mixer.stop()
            # but stopping the specific sound is more targeted.

    # --- End Sonar Sweep Animation Logic ---
    # --- End Sonar Sweep Parameter Calculation ---

    for evento in pygame.event.get():  # El usuario hizo algo
        if evento.type == pygame.QUIT: # Si el usuario hace click sobre cerrar
            hecho = True               # Marca que ya lo hemos hecho, de forma que abandonamos el bucle
        elif evento.type == pygame.VIDEORESIZE:
            dimensiones[0] = evento.w
            dimensiones[1] = evento.h
            # screen = pygame.display.set_mode(dimensiones, pygame.RESIZABLE) # Re-crear screen
            # No es estrictamente necesario re-crear `screen` aquí si solo actualizamos `dimensiones`
            # y todos los cálculos de UI en el bucle principal dependen de `dimensiones`.
            # Pygame maneja internamente la redimensión de la superficie devuelta por set_mode con RESIZABLE.
            # Lo importante es que nuestros cálculos usen las nuevas `dimensiones`.
            print(f"Ventana redimensionada a: {dimensiones}") # Debug
        
        if evento.type == pygame.KEYDOWN:
            # Ensure current_range_index is valid before accessing presets for S_max
            if current_range_index >= len(range_presets_map[current_unit]):
                current_range_index = len(range_presets_map[current_unit]) - 1
            s_max_for_event = range_presets_map[current_unit][current_range_index]

            handle_key_events(evento.key, 
                              circle_center_x, circle_center_y, 
                              display_radius_pixels, s_max_for_event) # Call the new function with new params
        
        if evento.type == pygame.MOUSEBUTTONDOWN:
            if evento.button == 1: # Left mouse button
                if not show_unidades_popup and not show_puerto_popup and not show_main_menu_popup: # Process main screen buttons only if ALL popups are closed
                    if button_unidades_rect.collidepoint(evento.pos):
                        show_unidades_popup = True 
                    elif button_puerto_rect.collidepoint(evento.pos): 
                        show_puerto_popup = True
                        available_com_ports_list = get_available_com_ports()
                        selected_com_port_in_popup = puerto if puerto else (available_com_ports_list[0] if available_com_ports_list else None)
                        selected_baud_rate_in_popup = baudios
                        puerto_popup_message = ""
                        show_com_port_dropdown = False 
                        show_baud_rate_dropdown = False
                    elif button_menu_rect.collidepoint(evento.pos): 
                        show_main_menu_popup = True # Open main menu
                        # Other popups are already confirmed closed by outer if
                        # Reset individual dropdown states when opening main menu
                        for key in menu_dropdown_states:
                            menu_dropdown_states[key] = False
                
                elif show_main_menu_popup: # Main Menu Pop-up is showing
                    # --- Click Handling for Main Menu ---
                    # First, check if the click was on the main menu button itself (to toggle it off)
                    if button_menu_rect.collidepoint(evento.pos):
                        show_main_menu_popup = False
                        # Resetting dropdowns when menu is closed by its button
                        for key in menu_dropdown_states:
                            menu_dropdown_states[key] = False
                        # No further click processing needed for this event if menu button was clicked
                    else:
                        clicked_on_menu_element = False
                        if 'interactive_menu_item_rects' in globals():
                            for option_key, rect_info in interactive_menu_item_rects.items():
                                if option_key in menu_dropdown_states and rect_info and 'value_box' in rect_info and 'items' in rect_info and 'item_values' in rect_info:
                                    if handle_menu_dropdown_click(evento.pos, option_key,
                                                                  rect_info['value_box'],
                                                                  rect_info['items'],
                                                                  rect_info['item_values']):
                                        clicked_on_menu_element = True
                                        break 
                                elif option_key in menu_options_values and not (option_key in menu_dropdown_states): 
                                    if rect_info and 'squares' in rect_info and 'values' in rect_info:
                                        if handle_square_selector_click(evento.pos, option_key,
                                                                        rect_info['squares'],
                                                                        rect_info['values']):
                                            clicked_on_menu_element = True
                                            break 
                        
                        if not clicked_on_menu_element:
                            if 'global_menu_panel_rect' in globals() and global_menu_panel_rect and \
                               not global_menu_panel_rect.collidepoint(evento.pos): 
                                    show_main_menu_popup = False
                                    for key in menu_dropdown_states: # Also reset dropdowns
                                        menu_dropdown_states[key] = False
                        # --- End Click Handling for Main Menu ---

                elif show_unidades_popup: # Unidades Pop-up is showing
                    if popup_metros_option_rect.collidepoint(evento.pos):
                        current_unit = "METERS"
                        try:
                            default_range_value_meters = 300 
                            current_range_index = RANGE_PRESETS_METERS.index(default_range_value_meters)
                        except ValueError:
                            current_range_index = 4 
                        active_sonar_rose_unit_surface = font.render(sonar_rose_unit_text_map[current_unit], True, BLANCO)
                        show_unidades_popup = False
                    elif popup_brazas_option_rect.collidepoint(evento.pos):
                        current_unit = "BRAZAS"
                        try:
                            default_range_value_brazas = 150 
                            current_range_index = RANGE_PRESETS_BRAZAS.index(default_range_value_brazas)
                        except ValueError:
                            current_range_index = 4 
                        active_sonar_rose_unit_surface = font.render(sonar_rose_unit_text_map[current_unit], True, BLANCO)
                        show_unidades_popup = False
                    elif not popup_main_rect.collidepoint(evento.pos) and not button_unidades_rect.collidepoint(evento.pos): 
                        show_unidades_popup = False
                
                elif show_puerto_popup: # Puerto Pop-up is showing
                    if puerto_popup_apply_button_rect.collidepoint(evento.pos):
                        if ser:
                            try:
                                ser.close()
                            except Exception as e:
                                print(f"Error closing previous serial port: {e}")
                        ser = None 
                        serial_port_available = False
                        puerto_popup_message = "Aplicando..." 

                        if selected_com_port_in_popup and selected_baud_rate_in_popup:
                            try:
                                temp_ser = serial.Serial(selected_com_port_in_popup, selected_baud_rate_in_popup, timeout=1)
                                ser = temp_ser 
                                puerto = selected_com_port_in_popup 
                                baudios = selected_baud_rate_in_popup 
                                serial_port_available = True
                                puerto_popup_message = f"Conectado a {puerto}@{baudios}"
                            except serial.SerialException as e:
                                print(f"Error opening serial port {selected_com_port_in_popup}: {e}")
                                puerto_popup_message = f"Error: {e}"
                                ser = None
                                serial_port_available = False
                            except Exception as e: 
                                print(f"Unexpected error connecting to {selected_com_port_in_popup}: {e}")
                                puerto_popup_message = f"Error inesperado: {e}"
                                ser = None
                                serial_port_available = False
                        else:
                            puerto_popup_message = "Seleccione puerto y baudios"
                        
                        puerto_popup_auto_close_start_time = pygame.time.get_ticks() 

                    elif puerto_popup_cancel_button_rect.collidepoint(evento.pos):
                        show_puerto_popup = False
                        show_com_port_dropdown = False
                        show_baud_rate_dropdown = False
                    elif puerto_popup_select_port_rect.collidepoint(evento.pos):
                        show_com_port_dropdown = not show_com_port_dropdown
                        show_baud_rate_dropdown = False 
                    elif puerto_popup_select_baud_rect.collidepoint(evento.pos):
                        show_baud_rate_dropdown = not show_baud_rate_dropdown
                        show_com_port_dropdown = False 
                    else:
                        clicked_outside_active_elements = True
                        if show_com_port_dropdown:
                            for i, item_rect in enumerate(puerto_popup_port_list_item_rects):
                                if item_rect.collidepoint(evento.pos):
                                    selected_com_port_in_popup = available_com_ports_list[i]
                                    show_com_port_dropdown = False
                                    clicked_outside_active_elements = False
                                    break
                            if not puerto_popup_select_port_rect.collidepoint(evento.pos) and clicked_outside_active_elements:
                                show_com_port_dropdown = False 
                                if not puerto_popup_main_rect.collidepoint(evento.pos):
                                   show_puerto_popup = False


                        if show_baud_rate_dropdown: 
                            for i, item_rect in enumerate(puerto_popup_baud_list_item_rects):
                                if item_rect.collidepoint(evento.pos):
                                    selected_baud_rate_in_popup = available_baud_rates_list[i]
                                    show_baud_rate_dropdown = False
                                    clicked_outside_active_elements = False
                                    break
                            if not puerto_popup_select_baud_rect.collidepoint(evento.pos) and clicked_outside_active_elements:
                                show_baud_rate_dropdown = False
                                if not puerto_popup_main_rect.collidepoint(evento.pos):
                                   show_puerto_popup = False
                        
                        if clicked_outside_active_elements and \
                           not puerto_popup_main_rect.collidepoint(evento.pos) and \
                           not button_puerto_rect.collidepoint(evento.pos): 
                            show_puerto_popup = False
                            show_com_port_dropdown = False
                            show_baud_rate_dropdown = False
                # --- End Puerto Pop-up Event Handling --- (This comment was slightly misplaced, fixed)


    # Read from serial port if available
    if serial_port_available and ser: # Check ser object directly
        try:
            if ser.in_waiting > 0: # Check in_waiting only if port seems available
                line = ser.readline().decode('ascii', errors='replace').strip()
                if line: # Process only if line is not empty
                    if line.startswith('$GPGLL') or line.startswith('$GNGLL'):
                        parse_gll(line)
                    elif line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                        parse_gga(line)
                    elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                        parse_rmc(line)
                    elif line.startswith('$GPVTG') or line.startswith('$GNVTG'):
                        parse_vtg(line)
                    elif line.startswith('$GPHDT') or line.startswith('$GNHDT'):
                        parse_hdt(line)
                    elif line.startswith('$GPHDG') or line.startswith('$GNHDG'):
                        parse_hdg(line)
                    elif line.startswith('$GPZDA') or line.startswith('$GNZDA'): # Common talkers for ZDA
                        parse_zda(line)
                    elif line.startswith('$PFEC,GPatt'): # Specific check for the proprietary sentence
                        parse_fec_gpatt(line)
        except serial.SerialException as se:
            print(f"COM port error/disconnected: {se}")
            if ser and ser.is_open:
                try:
                    ser.close()
                except Exception as close_ex:
                    print(f"Error closing COM port: {close_ex}")
            ser = None
            serial_port_available = False
            # Optionally, could update a global status message here for UI
            # e.g., global_ui_message = "COM Port Disconnected"
        except Exception as e: # Catch other potential errors like parsing errors, etc.
            print(f"Error processing serial data: {e}")
            # For general errors, we might not want to disable the port,
            # unless they are recurrent and indicate a fundamental issue.
            # For now, just print and continue.

    # --- Automatic COM Port Reconnection Attempt ---
    if not serial_port_available and puerto is not None: # 'puerto' holds the last known good port
        current_pygame_time = pygame.time.get_ticks()
        if current_pygame_time - last_reconnect_attempt_time > RECONNECT_INTERVAL_MS:
            last_reconnect_attempt_time = current_pygame_time
            # print(f"DEBUG: Attempting auto-reconnect to {puerto}@{baudios}...") # Optional for debugging
            try:
                if ser and ser.is_open: # Try to close if it somehow still exists and is open
                    try: ser.close()
                    except Exception: pass 
                ser = None # Ensure clean state

                temp_ser = serial.Serial(puerto, baudios, timeout=1)
                ser = temp_ser # Assign to global ser
                serial_port_available = True # If successful, this gets set to True
                print(f"INFO: Puerto COM {puerto} reconectado automáticamente.")
            except serial.SerialException:
                # Auto-reconnect failed, try again later. This is a silent failure.
                pass
            except Exception as e_reconnect:
                # Other unexpected error during auto-reconnect attempt.
                # print(f"DEBUG: Unexpected error during auto-reconnect: {e_reconnect}") # Optional
                pass
    # --- End Automatic Reconnection Attempt ---

    # --- Marker 'Screen' to 'Geo' Conversion on NMEA Activation ---
    # This logic needs to run *after* a potential auto-reconnect might make serial_port_available True
    if serial_port_available and not prev_serial_port_available and \
       current_ship_lat_deg is not None and current_ship_lon_deg is not None:
        # NMEA data just became available with a valid fix (either by manual or auto-reconnect)
        # print("DEBUG: NMEA activated, attempting to convert 'screen' markers to 'geo'.") # Debug
        for marker in target_markers:
            if marker['mode'] == 'screen' and marker.get('screen_bearing_rad') is not None \
               and marker.get('initial_distance_meters') is not None: # Now using initial_distance_meters
                
                # print(f"DEBUG: Converting marker: {marker}") # Debug
                screen_bearing_deg = math.degrees(marker['screen_bearing_rad'])
                # true_marker_bearing_deg is relative to true North
                # current_ship_heading is true heading. screen_bearing_deg is relative to ship's current screen up.
                true_marker_bearing_deg = (current_ship_heading + screen_bearing_deg + 360) % 360
                
                distance_m = marker['initial_distance_meters'] # Already in meters
                
                # print(f"DEBUG: ship_heading={current_ship_heading}, screen_bearing_deg={screen_bearing_deg}, true_marker_bearing={true_marker_bearing_deg}, dist_m={distance_m}") # Debug

                try:
                    ship_pos_at_conversion = Point(latitude=current_ship_lat_deg, longitude=current_ship_lon_deg)
                    destination = geodesic(meters=distance_m).destination(point=ship_pos_at_conversion, bearing=true_marker_bearing_deg)
                    
                    marker['geo_pos'] = {'lat': destination.latitude, 'lon': destination.longitude}
                    marker['mode'] = 'geo'
                    # print(f"DEBUG: Marker converted to GEO. New geo_pos: {marker['geo_pos']}") # Debug
                    # Optionally, clear screen-specific fields if they are strictly no longer needed
                    # marker.pop('screen_bearing_rad', None) # Keep for potential future reversion logic?
                    # marker.pop('initial_distance_units', None) # Keep for potential future reversion logic?
                except Exception as geo_calc_e:
                    print(f"Error calculating geo_pos for marker conversion: {geo_calc_e}")


    # --- End Marker Conversion ---

    # --- Reset NMEA display data if port/NMEA fix is lost ---
    if not serial_port_available or current_ship_lat_deg is None:
        latitude_str = "N/A"
        longitude_str = "N/A"
        speed_str = "N/A"
        # heading_str refers to the text display in the panel.
        # current_ship_heading is the float value for calculations and rose display.
        heading_str = "N/A" 
        current_ship_heading = 0.0 # Default heading for rose when NMEA is lost
        
        zda_time_str = "N/A"
        zda_date_str = "N/A"
        att_pitch_str = "N/A"
        att_roll_str = "N/A"
    # --- End Reset NMEA display data ---

    # Update previous serial port availability for the *next* frame's logic
    prev_serial_port_available = serial_port_available

    # Limpia la pantalla y establece su color de fondo
    pantalla.fill(current_colors["BACKGROUND"])


    # Usar las variables calculadas dinámicamente para el círculo del sonar
    dimensiones_caja = [circle_origin_x, circle_origin_y, circle_width, circle_height]
    # Dibujamos el borde de un círculo para 'barrerlo'
    pygame.draw.ellipse(pantalla, current_colors["PRIMARY_TEXT"], dimensiones_caja, 2)



    # The existing drawing code already calculates center_x, center_y, main_radius from dimensiones_caja:
    center_x = dimensiones_caja[0] + dimensiones_caja[2] // 2 # This is 310
    center_y = dimensiones_caja[1] + dimensiones_caja[3] // 2 # This is 310
    main_radius = dimensiones_caja[2] // 2 # This is 300

    radii = [main_radius * 1/4, main_radius * 2/4, main_radius * 3/4]

    for r in radii:
        # Convert radius to int for pixel dimensions, though pygame might handle floats.
        r_int = int(r) 
        ring_dims = [center_x - r_int, center_y - r_int, 2 * r_int, 2 * r_int]
        # pygame.draw.ellipse(pantalla, GRIS_MEDIO, ring_dims, 2) # Changed color to GRIS_MEDIO
        draw_dotted_ellipse(pantalla, current_colors["RANGE_RINGS"], ring_dims, dot_radius=1, spacing_angle=5) # Changed to GRIS_MUY_CLARO


    # Draw the compass rose
    # Assuming 'font' is the desired font for cardinal labels.
    # You might want to use a specific font size for clarity.
    draw_compass_rose(pantalla, center_x, center_y, main_radius, font, current_colors["COMPASS_ROSE"], current_ship_heading)

    # --- Draw Sonar Sweep ---
    if current_sweep_radius_pixels > 0 and current_sweep_radius_pixels <= display_radius_pixels:
        # Draw a semi-transparent blue circle for the sweep
        # To make it semi-transparent, we create a temporary surface
        sweep_surface = pygame.Surface((display_radius_pixels * 2, display_radius_pixels * 2), pygame.SRCALPHA)
        sweep_color_with_alpha = (*current_colors["SWEEP"], 150)
        pygame.draw.circle(sweep_surface, sweep_color_with_alpha,  # Changed AZUL to CELESTE, alpha 150
                           (display_radius_pixels, display_radius_pixels), # Center of the temp surface
                           int(current_sweep_radius_pixels), 2) # Thickness 2
        pantalla.blit(sweep_surface, (center_x - display_radius_pixels, center_y - display_radius_pixels))
    # --- End Draw Sonar Sweep ---

    # Draw the center icon
    draw_center_icon(pantalla, center_x, center_y, 36, current_colors["CENTER_ICON"]) # Changed height to 36, thickness remains 5

    # --- End Display Calculated Target Data ---

    # --- Draw Sonar Rose Unit Text (Bottom-Right of Sonar Rose, above target data if visible) ---
    # Re-render active_sonar_rose_unit_surface with current colors
    active_sonar_rose_unit_surface = font.render(sonar_rose_unit_text_map[current_unit], True, current_colors["PRIMARY_TEXT"])

    if active_sonar_rose_unit_surface:
        sonar_rose_unit_rect = active_sonar_rose_unit_surface.get_rect()
        
        # Position near the bottom-right of the sonar circle (dimensiones_caja defines sonar circle)
        # Default X: right edge of sonar circle, minus some padding
        base_x_unit_text = dimensiones_caja[0] + dimensiones_caja[2] - 10 # 10px from right edge of sonar
        
        # Default Y: bottom edge of sonar circle, minus some padding
        base_y_unit_text = dimensiones_caja[1] + dimensiones_caja[3] - 5 # 5px from bottom edge of sonar
        
        sonar_rose_unit_rect.bottomright = (base_x_unit_text, base_y_unit_text)

        # If target data is visible and its rect (td_dist_t1_t2_rect) is defined,
        # position unit text above the target data block.
        # td_dist_t1_t2_rect is the rect of the first line of the target data block.
        if len(target_markers) >= 2 and 'td_dist_t1_t2_rect' in locals() and td_dist_t1_t2_rect is not None:
            # Place it above the td_dist_t1_t2_rect (which is the first line of target data)
            # Align its right with the right of td_dist_t1_t2_rect or slightly offset if needed.
            # For simplicity, let's align with the right edge of sonar circle still, but adjust Y.
            sonar_rose_unit_rect.right = base_x_unit_text # Keep X alignment consistent
            sonar_rose_unit_rect.bottom = td_dist_t1_t2_rect.top - 3 # 3px margin above target data
            
        # Final check to ensure it's within sonar circle bounds (approx)
        if sonar_rose_unit_rect.right > dimensiones_caja[0] + dimensiones_caja[2] - 5:
             sonar_rose_unit_rect.right = dimensiones_caja[0] + dimensiones_caja[2] - 5
        if sonar_rose_unit_rect.bottom > dimensiones_caja[1] + dimensiones_caja[3] - 5:
             sonar_rose_unit_rect.bottom = dimensiones_caja[1] + dimensiones_caja[3] - 5
        if sonar_rose_unit_rect.left < dimensiones_caja[0] + 5:
             sonar_rose_unit_rect.left = dimensiones_caja[0] + 5
        if sonar_rose_unit_rect.top < dimensiones_caja[1] + 5:
             sonar_rose_unit_rect.top = dimensiones_caja[1] + 5

        pantalla.blit(active_sonar_rose_unit_surface, sonar_rose_unit_rect)
    # --- End Sonar Rose Unit Text ---

    # Display current heading at the top of the sonar circle
    heading_text_str = f"{int(current_ship_heading)}°" 
    heading_text_surface = font_very_large.render(heading_text_str, True, current_colors["PRIMARY_TEXT"]) 
    heading_text_rect = heading_text_surface.get_rect()
    heading_text_rect.centerx = center_x # center_x is circle_center_x
    heading_text_rect.top = circle_origin_y + 5 
    pantalla.blit(heading_text_surface, heading_text_rect)

    # Unified data box on the right (already adjusted dimensions - unified_data_box_dims)
    pygame.draw.rect(pantalla, current_colors["DATA_PANEL_BG"], unified_data_box_dims)
    pygame.draw.rect(pantalla, current_colors["DATA_PANEL_BORDER"], unified_data_box_dims, 2)

  # Slot 1: VELOCIDAD
    text_surface_longitud = font.render(texto_longitud, True, current_colors["PRIMARY_TEXT"]) # texto_longitud is "VELOC DEL BARCO"
    text_rect_longitud = text_surface_longitud.get_rect()
    text_rect_longitud.left = unified_data_box_dims[0] + 5 
    text_rect_longitud.top = unified_data_box_dims[1] + 5 
    screen.blit(text_surface_longitud, text_rect_longitud)

    if speed_str == "N/A":
        display_speed_text = "N/A"
    else:
        # Assuming speed_str is like "X.X Knots" or just "X.X" if NMEA is different
        numeric_part_speed = speed_str.replace(" Knots", "").strip()
        display_speed_text = f"{numeric_part_speed} kn"

    text_surface_speed_data = font_size_54.render(display_speed_text, True, current_colors["PRIMARY_TEXT"]) # Changed to font_size_54
    text_rect_speed_data = text_surface_speed_data.get_rect()
    text_rect_speed_data.right = unified_data_box_dims[0] + unified_data_box_dims[2] - 5
    text_rect_speed_data.bottom = unified_data_box_dims[1] + 100 - 8 
    screen.blit(text_surface_speed_data, text_rect_speed_data)

    # Slot 2: RUMBO
    y_offset_rumbo = unified_data_box_dims[1] + 100 + 5 # Start of RUMBO's 100px section + 5px padding from VELOCIDAD section

    text_surface_velocidad = font.render(texto_velocidad, True, current_colors["PRIMARY_TEXT"]) # texto_velocidad is "RUMBO DEL BARCO"
    text_rect_velocidad = text_surface_velocidad.get_rect()
    text_rect_velocidad.left = unified_data_box_dims[0] + 5
    text_rect_velocidad.top = y_offset_rumbo + 5 # 5px from top of its allocated section
    screen.blit(text_surface_velocidad, text_rect_velocidad)

    display_heading_text = "N/A" # Default
    if heading_str == "N/A":
        display_heading_text = "N/A"
    else:
        numeric_part_heading = ""
        for char_h in heading_str: # Extract leading numeric part
            if char_h.isdigit() or char_h == '.':
                numeric_part_heading += char_h
            else:
                break 

        if numeric_part_heading: # If we got a numeric string
            try:
                heading_value = int(float(numeric_part_heading))
                display_heading_text = f"{heading_value}°"
            except ValueError:
                display_heading_text = heading_str # Fallback on conversion error
        else: # If no numeric string was extracted (e.g., heading_str was non-numeric)
            display_heading_text = heading_str # Fallback

    text_surface_heading_data = font_size_54.render(display_heading_text, True, current_colors["PRIMARY_TEXT"]) # Changed to font_size_54
    text_rect_heading_data = text_surface_heading_data.get_rect()
    text_rect_heading_data.right = unified_data_box_dims[0] + unified_data_box_dims[2] - 5
    text_rect_heading_data.bottom = y_offset_rumbo + 100 - 8 
    screen.blit(text_surface_heading_data, text_rect_heading_data)

    # --- Slot 3: COORDENADAS/LAT/LON ---
    # Create all surfaces and get their initial rects first
    text_surface_latitud = font.render(texto_latitud, True, current_colors["PRIMARY_TEXT"])
    text_rect_latitud = text_surface_latitud.get_rect()

    text_surface_lat_data = font_size_54.render(latitude_str, True, current_colors["PRIMARY_TEXT"]) # Changed to font_size_54
    text_rect_lat_data = text_surface_lat_data.get_rect()

    text_surface_lon_data = font_size_54.render(longitude_str, True, current_colors["PRIMARY_TEXT"]) # Changed to font_size_54
    text_rect_lon_data = text_surface_lon_data.get_rect()

    # Set all horizontal positions
    text_rect_latitud.left = unified_data_box_dims[0] + 5 
    text_rect_lat_data.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2 
    text_rect_lon_data.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2 

    # Define Y offset for this section
    y_offset_latlon = unified_data_box_dims[1] + 210 # This is the start Y for the LAT/LON content block

    # Set vertical positions
    text_rect_latitud.top = y_offset_latlon + 3 # Title position
    
    # Position Longitude data first towards the bottom of its allocated space
    text_rect_lon_data.bottom = y_offset_latlon + 130 - 5 # Adjusted bottom padding slightly for larger font

    # Position Latitude data above Longitude data, with a small gap
    text_rect_lat_data.bottom = text_rect_lon_data.top - 5 # Adjusted gap for larger font

    # Blit all surfaces in visual order
    screen.blit(text_surface_latitud, text_rect_latitud)
    screen.blit(text_surface_lat_data, text_rect_lat_data)
    screen.blit(text_surface_lon_data, text_rect_lon_data)

    # Slot 4: DATO NUEVO 1 - HORA / FECHA
    # Content from the original combined_data_box_dims starts after the first three slots.
    # Height of first 3 slots: speed (100), course (100), latlon (140, approx from y_offset_latlon to its content bottom)
    # display_box_1_dims height was 340.
    section2_start_y = unified_data_box_dims[1] + 340 + 10 # unified_data_box_dims[1] is 10. 10 + 340 + 10 = 360.

    text_surface_db2_title = font.render(texto_dato_nuevo_1, True, current_colors["PRIMARY_TEXT"]) 
    text_rect_db2_title = text_surface_db2_title.get_rect()
    text_rect_db2_title.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2
    text_rect_db2_title.top = section2_start_y + 5
    screen.blit(text_surface_db2_title, text_rect_db2_title)

    text_surface_zda_time = font_data_medium.render(zda_time_str, True, current_colors["PRIMARY_TEXT"]) 
    text_rect_zda_time = text_surface_zda_time.get_rect()
    text_rect_zda_time.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2
    text_rect_zda_time.top = text_rect_db2_title.bottom + 3 
    screen.blit(text_surface_zda_time, text_rect_zda_time)

    text_surface_zda_date = font_data_medium.render(zda_date_str, True, current_colors["PRIMARY_TEXT"]) 
    text_rect_zda_date = text_surface_zda_date.get_rect()
    text_rect_zda_date.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2
    text_rect_zda_date.top = text_rect_zda_time.bottom + 3 
    screen.blit(text_surface_zda_date, text_rect_zda_date)

    # Slot 5: DATO NUEVO 3 - PITCH / ROLL (Moved up)
    y_start_pitch_roll = text_rect_zda_date.bottom + 18 # Start after HORA/FECHA (relative positioning)

    text_surface_db4_title = font.render(texto_dato_nuevo_3, True, current_colors["PRIMARY_TEXT"]) 
    text_rect_db4_title = text_surface_db4_title.get_rect()
    text_rect_db4_title.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2
    text_rect_db4_title.top = y_start_pitch_roll
    screen.blit(text_surface_db4_title, text_rect_db4_title)

    text_surface_att_pitch = font_data_medium.render(att_pitch_str, True, current_colors["PRIMARY_TEXT"]) 
    text_rect_att_pitch = text_surface_att_pitch.get_rect()
    text_rect_att_pitch.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2
    text_rect_att_pitch.top = text_rect_db4_title.bottom + 3 
    screen.blit(text_surface_att_pitch, text_rect_att_pitch)

    text_surface_att_roll = font_data_medium.render(att_roll_str, True, current_colors["PRIMARY_TEXT"]) 
    text_rect_att_roll = text_surface_att_roll.get_rect()
    text_rect_att_roll.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2
    text_rect_att_roll.top = text_rect_att_pitch.bottom + 3 
    screen.blit(text_surface_att_roll, text_rect_att_roll)
    
    # Slot 6: DATO NUEVO 2 - RATE OF TURN -- REMOVED
    # text_surface_db3_title = font.render(texto_dato_nuevo_2, True, current_colors["PRIMARY_TEXT"]) # texto_dato_nuevo_2 is removed
    # text_rect_db3_title = text_surface_db3_title.get_rect()
    # text_rect_db3_title.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2
    # text_rect_db3_title.top = text_rect_att_roll.bottom + 18 
    # screen.blit(text_surface_db3_title, text_rect_db3_title)

    # text_surface_rot_data = font_data_medium.render(rot_str, True, BLANCO) # rot_str is removed
    # text_rect_rot_data = text_surface_rot_data.get_rect()
    # text_rect_rot_data.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2
    # text_rect_rot_data.top = text_rect_db3_title.bottom + 3
    # screen.blit(text_surface_rot_data, text_rect_rot_data)


    # --- Draw "UNIDADES" and "PUERTO" Buttons ---
    # Positioned below PITCH / ROLL data
    box_internal_padding = 15 # Define padding for items within the unified box for these buttons
    button_internal_padding_y = 5 # Padding inside the button text (height)
    # Note: button_internal_padding_x is not explicitly used for width calculation here,
    # as button width is derived from available space. Text is centered.
    
    # Calculate available width for two buttons side-by-side
    total_available_width_for_buttons = unified_data_box_dims[2] - (box_internal_padding * 2) 
    gap_between_main_buttons = 10
    button_width = (total_available_width_for_buttons - gap_between_main_buttons) // 2

    # UNIDADES Button
    # Re-render surface if None (e.g., after color change)
    if button_unidades_text_surface is None:
        button_unidades_text_surface = font.render("UNIDADES", True, current_colors["PRIMARY_TEXT"], current_colors["BUTTON_BG"])

    button_unidades_rect.width = button_width
    button_unidades_rect.height = button_unidades_text_surface.get_height() + (2 * button_internal_padding_y)
    button_unidades_rect.left = unified_data_box_dims[0] + box_internal_padding 
    button_unidades_rect.top = text_rect_att_roll.bottom + 15 # Relative positioning still works

    if not show_unidades_popup and not show_puerto_popup: # Hide if either popup is active over this area
        pygame.draw.rect(pantalla, current_colors["BUTTON_BG"], button_unidades_rect) 
        pygame.draw.rect(pantalla, current_colors["BUTTON_BORDER"], button_unidades_rect, 2) 
        text_blit_rect_unidades = button_unidades_text_surface.get_rect(center=button_unidades_rect.center)
        pantalla.blit(button_unidades_text_surface, text_blit_rect_unidades)

    # PUERTO Button
    if button_puerto_text_surface is None:
        button_puerto_text_surface = font.render("PUERTO", True, current_colors["PRIMARY_TEXT"], current_colors["BUTTON_BG"])

    button_puerto_rect.width = button_width
    button_puerto_rect.height = button_puerto_text_surface.get_height() + (2 * button_internal_padding_y)
    button_puerto_rect.left = button_unidades_rect.right + gap_between_main_buttons
    button_puerto_rect.top = button_unidades_rect.top # Align top with Unidades button

    if not show_puerto_popup and not show_unidades_popup: # Hide if either popup is active
        pygame.draw.rect(pantalla, current_colors["BUTTON_BG"], button_puerto_rect)
        pygame.draw.rect(pantalla, current_colors["BUTTON_BORDER"], button_puerto_rect, 1) 
        text_blit_rect_puerto = button_puerto_text_surface.get_rect(center=button_puerto_rect.center)
        pantalla.blit(button_puerto_text_surface, text_blit_rect_puerto)
    
    # --- Draw "MENÚ" Button ---
    if button_menu_text_surface is None:
        button_menu_text_surface = font.render("MENÚ", True, current_colors["PRIMARY_TEXT"], current_colors["BUTTON_BG"])

    # Positioned in the top-right corner of the screen.
    menu_button_padding_x = 10
    menu_button_padding_y = 5 
    
    button_menu_rect.width = button_menu_text_surface.get_width() + (2 * menu_button_padding_x)
    button_menu_rect.height = button_menu_text_surface.get_height() + (2 * menu_button_padding_y)
    # Asegurar que el botón de menú esté a la derecha del panel de datos si hay espacio,
    # o en el borde de la pantalla si el panel de datos es muy ancho.
    button_menu_rect.right = unified_data_box_dims[0] - 10 # A la izquierda del panel de datos
    if button_menu_rect.left < circle_origin_x + circle_width + 10: # Si invade el área del sonar
        button_menu_rect.right = dimensiones[0] - 10 # Pegado al borde derecho de la pantalla
    button_menu_rect.top = 10

    # Draw MENÚ button only if no other popups are obscuring its general area
    # (This simple check might need refinement if popups overlap precisely)
    if not show_puerto_popup and not show_unidades_popup: # and not show_main_menu_popup (if it shouldn't be drawn when open)
        pygame.draw.rect(pantalla, current_colors["BUTTON_BG"], button_menu_rect)
        pygame.draw.rect(pantalla, current_colors["BUTTON_BORDER"], button_menu_rect, 1)
        text_blit_rect_menu = button_menu_text_surface.get_rect(center=button_menu_rect.center)
        pantalla.blit(button_menu_text_surface, text_blit_rect_menu)
    # --- End Draw "MENÚ" Button ---

    # --- End Draw Main Buttons ---


    # --- Display Current Range ---
    active_presets = range_presets_map[current_unit]
    active_unit_suffix = range_display_suffix_map[current_unit]
    # Ensure current_range_index is valid for the active_presets list
    if current_range_index >= len(active_presets):
        current_range_index = len(active_presets) -1 # Cap it if out of bounds
    
    current_range_val_from_list = active_presets[current_range_index]
    label_value_spacing_rtg = 2 # Space between R/T/G label and its value

    # --- Display Current Range ---
    range_text_str = f"R {current_range_val_from_list}{active_unit_suffix}"
    range_surface = font_data_medium.render(range_text_str, True, current_colors["PRIMARY_TEXT"])
    range_rect = range_surface.get_rect()
    # Posicionar a la izquierda del círculo del sonar, si hay espacio, o encima del panel de datos.
    range_rect.right = circle_origin_x + circle_width + unified_data_box_dims[0] - (circle_origin_x + circle_width) - 10 # Margen de 10px a la izquierda del panel de datos
    # Asegurar que no se superponga con el botón de menú si este último está a la izquierda del panel
    if 'button_menu_rect' in locals() and button_menu_rect.left < unified_data_box_dims[0]:
        if range_rect.right > button_menu_rect.left -5:
            range_rect.right = button_menu_rect.left - 5
    if range_rect.left < circle_origin_x + circle_width + 5: # Si no hay espacio, moverlo
        range_rect.left = circle_origin_x + circle_width + 5
    range_rect.top = 10
    pantalla.blit(range_surface, range_rect)
    # --- End Display Current Range ---

    # --- Display Current Tilt (Top Right) ---
    tilt_text_str = f"T {current_tilt_angle}°"
    tilt_surface = font_data_medium.render(tilt_text_str, True, current_colors["PRIMARY_TEXT"])
    tilt_rect = tilt_surface.get_rect()
    tilt_rect.right = range_rect.right # Alinear con el display de Rango
    tilt_rect.top = range_rect.bottom + 5
    pantalla.blit(tilt_surface, tilt_rect)
    # --- End Display Current Tilt ---

    # --- Display Current Gain (Top Right) ---
    gain_text_str = f"G {current_gain:.1f}"
    gain_surface = font_data_medium.render(gain_text_str, True, current_colors["PRIMARY_TEXT"])
    gain_rect = gain_surface.get_rect()
    gain_rect.right = range_rect.right # Alinear con el display de Rango
    gain_rect.top = tilt_rect.bottom + 15
    pantalla.blit(gain_surface, gain_rect)
    # --- End Display Current Gain ---

    # --- Display Calculated Cursor Data (Top-Left, dentro del círculo del sonar si es posible) ---
    unit_suffix_for_display = sonar_rose_unit_text_map[current_unit] 
    
    # line1_text = f"H: {ui_state['cursor_H_proj_display']}{unit_suffix_for_display}" # No longer used directly
    # line2_text = f"S: {ui_state['cursor_S_range_display']}{unit_suffix_for_display} \u2198" # No longer used directly
    # line3_text = f"D: {ui_state['cursor_Depth_display']}{unit_suffix_for_display}" # Original full string for value - No longer used directly
    # line4_text_val = f"{ui_state['cursor_bearing_display']}°" # Original full string for value - No longer used directly

    # Data for cursor display: (Label, value_string_getter_or_value, suffix_for_label)
    cursor_info_lines = [
        ("⬊", f" {ui_state['cursor_S_range_display']}", ""), # S (was H)
        ("⮕", f" {ui_state['cursor_H_proj_display']}", ""), # H (was S)
        ("⭣", f" {ui_state['cursor_Depth_display']}", ""), 
        ("B", f" {ui_state['cursor_bearing_display']}°", "")
    ]

    current_cursor_y_offset = circle_origin_y + 10 # Start inside the sonar circle
    label_value_spacing = 2 
    cursor_info_left_margin = circle_origin_x + 10

    for label_str, value_str, label_suffix_str in cursor_info_lines:
        label_full_str = label_str + label_suffix_str
        
        current_label_font = font_large
        if label_str == "B":
            current_label_font = font 
            
        label_surf = current_label_font.render(label_full_str, True, current_colors["PRIMARY_TEXT"])
        value_surf = font.render(value_str, True, current_colors["PRIMARY_TEXT"]) 

        label_rect = label_surf.get_rect(topleft=(cursor_info_left_margin, current_cursor_y_offset))
        value_rect = value_surf.get_rect(left=label_rect.right + label_value_spacing, centery=label_rect.centery)
        
        # Ensure it doesn't go out of the sonar circle bounds (approx)
        if value_rect.right < circle_origin_x + circle_width - 10 and label_rect.bottom < circle_origin_y + circle_height -10:
            pantalla.blit(label_surf, label_rect)
            pantalla.blit(value_surf, value_rect)
        
        current_cursor_y_offset = label_rect.bottom + 3
    # --- End Display Calculated Cursor Data ---

    # --- Display Calculated Target Data (Bottom-Right inside Sonar Rose) ---
    large_symbol_font = font_large 
    label_font = font          
    
    if len(target_markers) >= 2:
        effective_line_height = large_symbol_font.get_linesize() 
        
        margin_bottom_sonar_circle = 10 
        margin_right_sonar_circle = 10
        symbol_value_spacing = 2

        target_data_lines_info = [
            ("⬌", f" {ui_state['target_dist_t1_t2']}", True),
            ("⮕", f" {ui_state['target_dist_center_t2']}", True),
            ("⭣", f" {ui_state['target_depth_t2']}", True),
            ("S", f" {ui_state['target_speed_t1_t2']}", False), 
            ("C", f" {ui_state['target_course_t1_t2']}", False)
        ]

        # Calculate total height to position the block from the bottom of the sonar circle
        num_lines_target_data = len(target_data_lines_info)
        # Approximate height: use label_font height as it's likely the smallest defining factor per line after symbols
        # This is a simplification; a more robust way sums actual rendered heights.
        # For now, using effective_line_height which is based on the larger font (large_symbol_font)
        # to ensure enough space if symbols are tall.
        total_text_block_height = (num_lines_target_data * effective_line_height) + \
                                  ((num_lines_target_data - 1) * 3 if num_lines_target_data > 0 else 0)


        # Start Y for the block, from the bottom of the sonar circle
        start_y_for_target_block = (circle_origin_y + circle_height) - total_text_block_height - margin_bottom_sonar_circle
        
        current_y_offset_target_data = start_y_for_target_block # Initialize y_offset for the first line

        max_line_width = 0
        rendered_lines = []
        for label_str, value_str, is_special in target_data_lines_info:
            font_for_label = large_symbol_font if is_special else label_font
            label_surf = font_for_label.render(label_str, True, current_colors["PRIMARY_TEXT"])
            value_surf = label_font.render(value_str, True, current_colors["PRIMARY_TEXT"]) # Values always small
            line_width = label_surf.get_width() + symbol_value_spacing + value_surf.get_width()
            if line_width > max_line_width:
                max_line_width = line_width
            rendered_lines.append({'label_surf': label_surf, 'value_surf': value_surf, 'is_special': is_special})

        # Start X for the block, from the right edge of the sonar circle
        start_x_for_target_block = (circle_origin_x + circle_width) - max_line_width - margin_right_sonar_circle


        for i, line_info in enumerate(rendered_lines): 
            label_surf = line_info['label_surf']
            value_surf = line_info['value_surf']
            
            # Position label part of the line
            current_line_total_width = label_surf.get_width() + symbol_value_spacing + value_surf.get_width()
            current_start_x_for_line = start_x_for_target_block + (max_line_width - current_line_total_width) # Right align this line within the block

            y_pos_for_this_line = current_y_offset_target_data
            
            # Store rect for the first line (td_dist_t1_t2_rect) if needed elsewhere (e.g. for Sonar Rose Unit Text positioning)
            # This part of the logic for td_dist_t1_t2_rect might need re-evaluation if its original purpose was
            # for positioning elements outside the sonar circle relative to this block.
            # Since this block is now *inside* the sonar circle, its absolute screen rect might not be as useful
            # for those external elements.
            # For now, we'll keep the calculation in case it's used for relative internal positioning or debugging.
            if i == 0: 
                 td_dist_t1_t2_rect = pygame.Rect(
                     current_start_x_for_line, 
                     y_pos_for_this_line, 
                     current_line_total_width, 
                     label_surf.get_height() # Or effective_line_height if all lines should have same conceptual height for this rect
                 )


            label_rect = label_surf.get_rect(topleft=(current_start_x_for_line, y_pos_for_this_line))
            value_rect = value_surf.get_rect(left=label_rect.right + symbol_value_spacing, centery=label_rect.centery) 
            
            # Ensure it doesn't go out of the sonar circle bounds (approx)
            if value_rect.right < circle_origin_x + circle_width - margin_right_sonar_circle + 5 and \
               label_rect.bottom < circle_origin_y + circle_height - margin_bottom_sonar_circle + 5 :
                 pantalla.blit(label_surf, label_rect)
                 pantalla.blit(value_surf, value_rect)
            
            current_y_offset_target_data += effective_line_height + 3 # Use consistent line height for spacing
    # --- End Display Calculated Target Data ---


    # --- Draw Custom "+" Cursor ---
    if ui_state["show_plus_cursor"]:
        cursor_x, cursor_y = ui_state["mouse_cursor_pos"]
        cursor_arm_length = 18 
        pygame.draw.line(pantalla, current_colors["CURSOR_CROSS"],
                         (cursor_x - cursor_arm_length, cursor_y), 
                         (cursor_x + cursor_arm_length, cursor_y), 2)
        pygame.draw.line(pantalla, current_colors["CURSOR_CROSS"],
                         (cursor_x, cursor_y - cursor_arm_length), 
                         (cursor_x, cursor_y + cursor_arm_length), 2)
    # --- End Draw Custom "+" Cursor ---

    # --- Draw Target Markers & Lines Between Them ---
    # The size of the rhombus and X mark should match the '+' cursor size.
    # The '+' cursor has arms of length `cursor_arm_length`, so total span is `cursor_arm_length * 2`.
    # However, `cursor_arm_length` is defined inside the `while not hecho` loop, 
    # so we need to ensure it's accessible or use its value (18*2 = 36 for size).
    # For simplicity here, we'll use the value directly, but ideally, it should be passed or stored globally.
    marker_icon_size = 18 # Reduced from 36

    for marker in target_markers:
        draw_pos = None
        if marker['mode'] == 'geo':
            if marker['current_screen_pos']: # Geo markers use dynamically updated screen_pos
                # Check if the geo marker is within the sonar circle for drawing
                mx_geo, my_geo = marker['current_screen_pos']
                dist_sq_to_center = (mx_geo - circle_center_x)**2 + (my_geo - circle_center_y)**2
                if dist_sq_to_center <= (display_radius_pixels + marker_icon_size/2)**2:
                    draw_pos = marker['current_screen_pos']
        
        elif marker['mode'] == 'screen':
            # For 'screen' mode markers, their position is determined by 'current_screen_pos',
            # which should have been updated by update_marker_screen_positions()
            # to reflect scaling if NMEA is off, or remain fixed if NMEA is on.
            draw_pos = marker['current_screen_pos']

        if draw_pos:
            mx, my = draw_pos
            current_marker_color = current_colors["TARGET_HOVER"] if marker.get('is_hovered', False) else current_colors["TARGET_BASE"]
            if marker['type'] == TARGET_TYPE_RHOMBUS:
                draw_rhombus(pantalla, current_marker_color, mx, my, marker_icon_size)
            elif marker['type'] == TARGET_TYPE_X:
                draw_x_mark(pantalla, current_marker_color, mx, my, marker_icon_size)

    # Draw lines between consecutive target markers
    for i in range(len(target_markers) - 1):
        marker1 = target_markers[i]
        marker2 = target_markers[i+1]

        pos1 = marker1['current_screen_pos']
        pos2 = marker2['current_screen_pos'] # This will be None if marker is off-screen due to range

        # s_max_disp_range is needed for 'screen' mode hypothetical point calculation.
        # Ensure current_range_index is valid.
        current_s_max_range_units = 0
        if 0 <= current_range_index < len(range_presets_map[current_unit]):
            current_s_max_range_units = range_presets_map[current_unit][current_range_index]
        else: # Should not happen if current_range_index is always maintained correctly
            current_s_max_range_units = range_presets_map[current_unit][0]


        if pos1 and pos2:
            # Both markers are on screen, draw a direct line.
            pygame.draw.line(pantalla, current_colors["PRIMARY_TEXT"], pos1, pos2, 1)
        
        elif pos1 and not pos2: # Marker1 is on screen, Marker2 is off screen (pos2 is None)
            hypothetical_pos2_far = None
            if marker2['mode'] == 'screen' and marker2.get('original_angle_rad') is not None:
                # SCREEN mode off-screen marker
                far_radius = display_radius_pixels * 10 
                # original_angle_rad for 'screen' markers is math.atan2(dy, dx) from center.
                # x = center + R * cos(angle), y = center + R * sin(angle)
                hypothetical_m2_x = circle_center_x + far_radius * math.cos(marker2['original_angle_rad'])
                hypothetical_m2_y = circle_center_y + far_radius * math.sin(marker2['original_angle_rad'])
                hypothetical_pos2_far = (hypothetical_m2_x, hypothetical_m2_y)
            
            elif marker2['mode'] == 'geo' and marker2.get('geo_pos') and \
                 current_ship_lat_deg is not None and current_ship_lon_deg is not None:
                # GEO mode off-screen marker
                ship_point = Point(latitude=current_ship_lat_deg, longitude=current_ship_lon_deg)
                target_point = Point(latitude=marker2['geo_pos']['lat'], longitude=marker2['geo_pos']['lon'])

                # Calculate true bearing from ship to target (consistent with update_marker_screen_positions)
                delta_lon_rad = math.radians(target_point.longitude - ship_point.longitude)
                lat1_rad = math.radians(ship_point.latitude)
                lat2_rad = math.radians(target_point.latitude)
                y_brg = math.sin(delta_lon_rad) * math.cos(lat2_rad)
                x_brg = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)
                true_bearing_to_target_rad = math.atan2(y_brg, x_brg)
                true_bearing_to_target_deg = (math.degrees(true_bearing_to_target_rad) + 360) % 360
                
                relative_bearing_deg = (true_bearing_to_target_deg - current_ship_heading + 360) % 360
                # angle_rad_for_draw: 0 is East, positive CCW. For screen: North is -PI/2.
                angle_rad_for_draw = math.radians(relative_bearing_deg - 90) 

                far_radius = display_radius_pixels * 10
                # Use cos for X, sin for Y because angle_rad_for_draw is adjusted for screen (-90 deg)
                # where 0 rad = East, PI/2 rad = South (if Y points down), -PI/2 rad = North (if Y points up)
                hypothetical_m2_x = circle_center_x + far_radius * math.cos(angle_rad_for_draw)
                hypothetical_m2_y = circle_center_y + far_radius * math.sin(angle_rad_for_draw) # Pygame Y is down, so positive sin is down
                hypothetical_pos2_far = (hypothetical_m2_x, hypothetical_m2_y)

            if hypothetical_pos2_far:
                dist_sq_pos1_from_center = (pos1[0] - circle_center_x)**2 + (pos1[1] - circle_center_y)**2
                if dist_sq_pos1_from_center <= (display_radius_pixels + 1)**2: # pos1 is inside or on edge
                    intersection_point = get_screen_line_circle_intersection(
                        pos1, hypothetical_pos2_far, 
                        circle_center_x, circle_center_y, 
                        display_radius_pixels - 0.5 
                    )
                    if intersection_point:
                        if math.sqrt((intersection_point[0]-pos1[0])**2 + (intersection_point[1]-pos1[1])**2) > 1:
                             pygame.draw.line(pantalla, current_colors["PRIMARY_TEXT"], pos1, intersection_point, 1)

        elif not pos1 and pos2: # Marker1 is off screen (pos1 is None), Marker2 is on screen
            hypothetical_pos1_far = None
            if marker1['mode'] == 'screen' and marker1.get('original_angle_rad') is not None:
                # SCREEN mode off-screen marker1
                far_radius = display_radius_pixels * 10
                hypothetical_m1_x = circle_center_x + far_radius * math.cos(marker1['original_angle_rad'])
                hypothetical_m1_y = circle_center_y + far_radius * math.sin(marker1['original_angle_rad'])
                hypothetical_pos1_far = (hypothetical_m1_x, hypothetical_m1_y)

            elif marker1['mode'] == 'geo' and marker1.get('geo_pos') and \
                 current_ship_lat_deg is not None and current_ship_lon_deg is not None:
                # GEO mode off-screen marker1
                ship_point = Point(latitude=current_ship_lat_deg, longitude=current_ship_lon_deg)
                target_point = Point(latitude=marker1['geo_pos']['lat'], longitude=marker1['geo_pos']['lon'])

                delta_lon_rad = math.radians(target_point.longitude - ship_point.longitude)
                lat1_rad = math.radians(ship_point.latitude)
                lat2_rad = math.radians(target_point.latitude)
                y_brg = math.sin(delta_lon_rad) * math.cos(lat2_rad)
                x_brg = math.cos(lat1_rad) * math.sin(lat2_rad) - \
                        math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)
                true_bearing_to_target_rad = math.atan2(y_brg, x_brg)
                true_bearing_to_target_deg = (math.degrees(true_bearing_to_target_rad) + 360) % 360
                
                relative_bearing_deg = (true_bearing_to_target_deg - current_ship_heading + 360) % 360
                angle_rad_for_draw = math.radians(relative_bearing_deg - 90)

                far_radius = display_radius_pixels * 10
                hypothetical_m1_x = circle_center_x + far_radius * math.cos(angle_rad_for_draw)
                hypothetical_m1_y = circle_center_y + far_radius * math.sin(angle_rad_for_draw)
                hypothetical_pos1_far = (hypothetical_m1_x, hypothetical_m1_y)

            if hypothetical_pos1_far:
                dist_sq_pos2_from_center = (pos2[0] - circle_center_x)**2 + (pos2[1] - circle_center_y)**2
                if dist_sq_pos2_from_center <= (display_radius_pixels + 1)**2: # pos2 is inside or on edge
                    intersection_point = get_screen_line_circle_intersection(
                        pos2, hypothetical_pos1_far, 
                        circle_center_x, circle_center_y,
                        display_radius_pixels - 0.5
                    )
                    if intersection_point:
                        if math.sqrt((intersection_point[0]-pos2[0])**2 + (intersection_point[1]-pos2[1])**2) > 1:
                            pygame.draw.line(pantalla, current_colors["PRIMARY_TEXT"], pos2, intersection_point, 1)

    # --- End Draw Target Markers & Lines ---

    # --- Draw Ship Track ---
    # Ensure current_range_index is valid before accessing presets for S_max
    if current_range_index >= len(range_presets_map[current_unit]):
        current_range_index = len(range_presets_map[current_unit]) - 1
    s_max_for_track_draw = range_presets_map[current_unit][current_range_index]
    
    draw_ship_track(pantalla, ship_track_points, 
                    current_ship_lat_deg, current_ship_lon_deg, current_ship_heading,
                    circle_center_x, circle_center_y, display_radius_pixels,
                    s_max_for_track_draw, current_unit, current_colors["SHIP_TRACK"])
    # --- End Draw Ship Track ---

    # --- Temporary Tilt Display on Sonar Circle (Proa) ---
    if show_tilt_temporarily:
        temp_tilt_text_str = f"T {current_tilt_angle}°" 
        temp_tilt_surface = font_data_medium.render(temp_tilt_text_str, True, current_colors["PRIMARY_TEXT"]) 
        temp_tilt_rect = temp_tilt_surface.get_rect()
        temp_tilt_rect.centerx = circle_center_x # Use circle_center_x consistently
        temp_tilt_rect.top = heading_text_rect.bottom + 5 
        
        # Ensure it's within the sonar circle
        if temp_tilt_rect.bottom < circle_origin_y + circle_height - 5:
            pantalla.blit(temp_tilt_surface, temp_tilt_rect)

        tilt_display_timer -= 1
        if tilt_display_timer <= 0:
            show_tilt_temporarily = False
    # --- End Temporary Tilt Display ---

    # --- Temporary Range Display on Sonar Circle (Proa) ---
    if show_range_temporarily:
        current_range_val_for_temp_display = range_presets_map[current_unit][current_range_index]
        temp_range_text_str = f"R {current_range_val_for_temp_display}{range_display_suffix_map[current_unit]}"
        temp_range_surface = font_data_medium.render(temp_range_text_str, True, current_colors["PRIMARY_TEXT"])
        temp_range_rect = temp_range_surface.get_rect()
        temp_range_rect.centerx = circle_center_x # Use circle_center_x

        base_y_position_temp_range = heading_text_rect.bottom
        if show_tilt_temporarily and 'temp_tilt_rect' in locals() and temp_tilt_rect is not None and temp_tilt_rect.bottom < circle_origin_y + circle_height - 5 :
             base_y_position_temp_range = temp_tilt_rect.bottom 
        temp_range_rect.top = base_y_position_temp_range + 5
        
        if temp_range_rect.bottom < circle_origin_y + circle_height - 5:
            pantalla.blit(temp_range_surface, temp_range_rect)

        range_display_timer -= 1
        if range_display_timer <= 0:
            show_range_temporarily = False
    # --- End Temporary Range Display ---

    # --- Temporary Gain Display on Sonar Circle (Proa) ---
    if show_gain_temporarily:
        temp_gain_text_str = f"G {current_gain:.1f}"
        temp_gain_surface = font_data_medium.render(temp_gain_text_str, True, current_colors["PRIMARY_TEXT"])
        temp_gain_rect = temp_gain_surface.get_rect()
        temp_gain_rect.centerx = circle_center_x # Use circle_center_x

        base_y_position_temp_gain = heading_text_rect.bottom
        # Check if Tilt is shown and displayed
        if show_tilt_temporarily and 'temp_tilt_rect' in locals() and temp_tilt_rect is not None and temp_tilt_rect.bottom < circle_origin_y + circle_height - 5 :
            base_y_position_temp_gain = temp_tilt_rect.bottom
        # Check if Range is shown (and was displayed) and is lower than Tilt (or Tilt not shown)
        if show_range_temporarily and 'temp_range_rect' in locals() and temp_range_rect is not None and temp_range_rect.bottom < circle_origin_y + circle_height - 5:
            if not (show_tilt_temporarily and 'temp_tilt_rect' in locals() and temp_tilt_rect is not None and temp_tilt_rect.bottom < circle_origin_y + circle_height - 5 and temp_range_rect.top <= temp_tilt_rect.bottom): # if range is not above or at same level as a displayed tilt
                 base_y_position_temp_gain = max(base_y_position_temp_gain, temp_range_rect.bottom)


        temp_gain_rect.top = base_y_position_temp_gain + 5
        
        if temp_gain_rect.bottom < circle_origin_y + circle_height - 5:
            pantalla.blit(temp_gain_surface, temp_gain_rect)

        gain_display_timer -= 1
        if gain_display_timer <= 0:
            show_gain_temporarily = False
    # --- End Temporary Gain Display ---

    # --- Draw Pop-up Window (if active) ---
    if show_unidades_popup:
        popup_width = 200 
        popup_height = 100 
        popup_main_rect.width = popup_width
        popup_main_rect.height = popup_height
        
        popup_main_rect.centerx = unified_data_box_dims[0] + unified_data_box_dims[2] // 2 # Use unified box
        # Position below the UNIDADES button's usual spot or related content
        # The UNIDADES button's top is text_rect_att_roll.bottom + 15.
        # This positioning keeps the popup starting at roughly the same vertical area as the buttons.
        if 'text_rect_att_roll' in locals() and text_rect_att_roll is not None: 
             popup_main_rect.top = text_rect_att_roll.bottom + 15 
        else: # Fallback if text_rect_att_roll somehow not defined
            # This fallback might need adjustment if text_rect_att_roll is critical for Y
            # A better fallback might be relative to where the buttons are drawn.
            # For now, keeping centery as a last resort.
            popup_main_rect.centery = pantalla.get_rect().centery 

        # Use GRIS_MEDIO for popup background, BLANCO for border (consistent across themes for now)
        pygame.draw.rect(pantalla, GRIS_MEDIO, popup_main_rect) 
        pygame.draw.rect(pantalla, BLANCO, popup_main_rect, 2)    

        button_height = 30 
        button_width = popup_width - 40 
        top_margin = 10 
        padding_between_buttons = 5 

        # Re-render text surfaces for popup if they are None (e.g. after color change)
        if popup_metros_text_surface is None:
            popup_metros_text_surface = font.render("METROS", True, BLANCO, NEGRO) # Popup buttons keep BLANCO/NEGRO
        if popup_brazas_text_surface is None:
            popup_brazas_text_surface = font.render("BRAZAS", True, BLANCO, NEGRO)


        popup_metros_option_rect.width = button_width
        popup_metros_option_rect.height = button_height
        popup_metros_option_rect.centerx = popup_main_rect.centerx
        popup_metros_option_rect.top = popup_main_rect.top + top_margin

        pygame.draw.rect(pantalla, NEGRO, popup_metros_option_rect) 
        pygame.draw.rect(pantalla, BLANCO, popup_metros_option_rect, 1) 
        metros_text_blit_rect = popup_metros_text_surface.get_rect(center=popup_metros_option_rect.center)
        pantalla.blit(popup_metros_text_surface, metros_text_blit_rect)

        popup_brazas_option_rect.width = button_width
        popup_brazas_option_rect.height = button_height
        popup_brazas_option_rect.centerx = popup_main_rect.centerx
        popup_brazas_option_rect.top = popup_metros_option_rect.bottom + padding_between_buttons

        pygame.draw.rect(pantalla, NEGRO, popup_brazas_option_rect) 
        pygame.draw.rect(pantalla, BLANCO, popup_brazas_option_rect, 1) 
        brazas_text_blit_rect = popup_brazas_text_surface.get_rect(center=popup_brazas_option_rect.center)
        pantalla.blit(popup_brazas_text_surface, brazas_text_blit_rect)
    # --- End Draw Pop-up Window ---

    # --- Draw "Puerto" Pop-up Window (if active) ---
    if show_puerto_popup:
        # Re-render surfaces for Puerto popup if they are None
        if puerto_popup_title_surf is None:
            puerto_popup_title_surf = font_large.render("Configurar Puerto", True, BLANCO) # Popup text BLANCO
        if puerto_popup_com_label_surf is None:
            puerto_popup_com_label_surf = font.render("Puerto COM:", True, BLANCO)
        if puerto_popup_baud_label_surf is None:
            puerto_popup_baud_label_surf = font.render("Baudios:", True, BLANCO)
        if puerto_popup_apply_text_surf is None:
            puerto_popup_apply_text_surf = font.render("Aplicar", True, BLANCO, NEGRO) # Button text BLANCO/NEGRO
        if puerto_popup_cancel_text_surf is None:
            puerto_popup_cancel_text_surf = font.render("Cancelar", True, BLANCO, NEGRO)


        # Define pop-up size and position
        base_popup_width = 350  # Desired width for content
        popup_height = 280      # Fixed height
        
        # Position the pop-up to align with the start of the second section of content
        # in the new unified_data_box.
        # section2_start_y was defined earlier as: unified_data_box_dims[1] + 340 + 10
        # Ensure section2_start_y is accessible or recalculate if necessary.
        # For robustness, let's use the y-coordinate of where the HORA/FECHA title (text_rect_db2_title) is placed,
        # minus its top padding, if available. Or recalculate section2_start_y.
        # text_rect_db2_title.top = section2_start_y + 5
        # So, section2_start_y = text_rect_db2_title.top - 5 (if text_rect_db2_title is already set)
        # This assumes text_rect_db2_title is defined before this popup drawing logic. It is.
        
        popup_x = unified_data_box_dims[0]
        if 'text_rect_db2_title' in locals() and text_rect_db2_title is not None:
            popup_y = text_rect_db2_title.top - 5 # Align with the start of HORA/FECHA block
        else: # Fallback if text_rect_db2_title not yet defined (should not happen in normal flow)
            popup_y = unified_data_box_dims[1] + 340 + 10 # Recalculate section2_start_y

        # Adjust width if it would go off-screen
        max_allowed_width = pantalla.get_width() - popup_x - 5 # 5px margin from screen edge
        actual_popup_width = min(base_popup_width, max_allowed_width)
        actual_popup_width = max(actual_popup_width, 200) # Example minimum width

        puerto_popup_main_rect.width = actual_popup_width
        puerto_popup_main_rect.height = popup_height # Keep original desired height

        # Adjust vertical position if it goes off bottom
        if popup_y + puerto_popup_main_rect.height > pantalla.get_height() - 5: # 5px margin
            popup_y = pantalla.get_height() - 5 - puerto_popup_main_rect.height
        
        puerto_popup_main_rect.topleft = (popup_x, popup_y)

        # Use GRIS_MEDIO for popup background, BLANCO for border (consistent across themes for now)
        pygame.draw.rect(pantalla, GRIS_MEDIO, puerto_popup_main_rect) 
        pygame.draw.rect(pantalla, BLANCO, puerto_popup_main_rect, 2)    

        title_rect = puerto_popup_title_surf.get_rect(centerx=puerto_popup_main_rect.centerx, top=puerto_popup_main_rect.top + 10)
        pantalla.blit(puerto_popup_title_surf, title_rect) # Text color is BLANCO from re-render

        current_y_offset_puerto = title_rect.bottom + 15 # Renamed to avoid conflict
        popup_internal_padding_x = 15 
        dropdown_height = 25
        dropdown_item_height = 20

        # COM Port Selection
        pantalla.blit(puerto_popup_com_label_surf, (puerto_popup_main_rect.left + popup_internal_padding_x, current_y_offset_puerto)) # Text color BLANCO
        puerto_popup_select_port_rect.topleft = (puerto_popup_main_rect.left + popup_internal_padding_x + puerto_popup_com_label_surf.get_width() + 10, current_y_offset_puerto -2)
        puerto_popup_select_port_rect.width = puerto_popup_main_rect.width - (popup_internal_padding_x * 2) - puerto_popup_com_label_surf.get_width() - 10
        puerto_popup_select_port_rect.height = dropdown_height
        pygame.draw.rect(pantalla, NEGRO, puerto_popup_select_port_rect) # Dropdown box NEGRO/BLANCO
        pygame.draw.rect(pantalla, BLANCO, puerto_popup_select_port_rect, 1)
        port_display_text = selected_com_port_in_popup if selected_com_port_in_popup else "Seleccionar..."
        port_display_surf = font.render(port_display_text, True, BLANCO) # Text in dropdown BLANCO
        pantalla.blit(port_display_surf, port_display_surf.get_rect(centery=puerto_popup_select_port_rect.centery, left=puerto_popup_select_port_rect.left + 5))

        current_y_offset_puerto += dropdown_height + 5 

        if show_com_port_dropdown:
            list_box_x = puerto_popup_select_port_rect.left
            list_box_y = puerto_popup_select_port_rect.bottom + 2
            list_box_width = puerto_popup_select_port_rect.width
            max_dropdown_list_height = 100 
            
            puerto_popup_port_list_item_rects.clear()
            temp_y_list_item = list_box_y
            actual_list_height = min(len(available_com_ports_list) * dropdown_item_height, max_dropdown_list_height)
            if not available_com_ports_list: actual_list_height = dropdown_item_height 
            
            pygame.draw.rect(pantalla, NEGRO, (list_box_x, list_box_y, list_box_width, actual_list_height)) # Dropdown list NEGRO/BLANCO
            pygame.draw.rect(pantalla, BLANCO, (list_box_x, list_box_y, list_box_width, actual_list_height),1)

            if not available_com_ports_list:
                no_ports_surf = font.render("No hay puertos disponibles", True, GRIS_MEDIO) # GRIS_MEDIO for disabled text
                pantalla.blit(no_ports_surf, no_ports_surf.get_rect(centerx=list_box_x + list_box_width // 2, top=temp_y_list_item + 2))
            else:
                for i, port_name in enumerate(available_com_ports_list):
                    if temp_y_list_item + dropdown_item_height > list_box_y + max_dropdown_list_height:
                        break 
                    item_rect = pygame.Rect(list_box_x, temp_y_list_item, list_box_width, dropdown_item_height)
                    puerto_popup_port_list_item_rects.append(item_rect)
                    item_color = BLANCO # Default item color BLANCO
                    if port_name == selected_com_port_in_popup: 
                        item_color = VERDE # Selected item VERDE
                    port_item_surf = font.render(port_name, True, item_color)
                    pantalla.blit(port_item_surf, port_item_surf.get_rect(centery=item_rect.centery, left=item_rect.left + 5))
                    temp_y_list_item += dropdown_item_height
            current_y_offset_puerto = list_box_y + actual_list_height + 10 
        else:
             current_y_offset_puerto = puerto_popup_select_port_rect.bottom + 10


        # Baud Rate Selection
        pantalla.blit(puerto_popup_baud_label_surf, (puerto_popup_main_rect.left + popup_internal_padding_x, current_y_offset_puerto)) # Text color BLANCO
        puerto_popup_select_baud_rect.topleft = (puerto_popup_main_rect.left + popup_internal_padding_x + puerto_popup_baud_label_surf.get_width() + 10, current_y_offset_puerto - 2)
        puerto_popup_select_baud_rect.width = puerto_popup_main_rect.width - (popup_internal_padding_x * 2) - puerto_popup_baud_label_surf.get_width() - 10
        puerto_popup_select_baud_rect.height = dropdown_height
        pygame.draw.rect(pantalla, NEGRO, puerto_popup_select_baud_rect) # Dropdown box NEGRO/BLANCO
        pygame.draw.rect(pantalla, BLANCO, puerto_popup_select_baud_rect, 1)
        baud_display_surf = font.render(str(selected_baud_rate_in_popup), True, BLANCO) # Text in dropdown BLANCO
        pantalla.blit(baud_display_surf, baud_display_surf.get_rect(centery=puerto_popup_select_baud_rect.centery, left=puerto_popup_select_baud_rect.left + 5))
        
        current_y_offset_puerto += dropdown_height + 5

        if show_baud_rate_dropdown:
            list_box_x = puerto_popup_select_baud_rect.left
            list_box_y = puerto_popup_select_baud_rect.bottom + 2
            list_box_width = puerto_popup_select_baud_rect.width
            max_dropdown_list_height = 100 
            
            puerto_popup_baud_list_item_rects.clear()
            temp_y_list_item = list_box_y
            actual_list_height = min(len(available_baud_rates_list) * dropdown_item_height, max_dropdown_list_height)
            pygame.draw.rect(pantalla, NEGRO, (list_box_x, list_box_y, list_box_width, actual_list_height)) # Dropdown list NEGRO/BLANCO
            pygame.draw.rect(pantalla, BLANCO, (list_box_x, list_box_y, list_box_width, actual_list_height),1)

            for i, baud_val in enumerate(available_baud_rates_list):
                if temp_y_list_item + dropdown_item_height > list_box_y + max_dropdown_list_height:
                    break 
                item_rect = pygame.Rect(list_box_x, temp_y_list_item, list_box_width, dropdown_item_height)
                puerto_popup_baud_list_item_rects.append(item_rect)
                item_color = BLANCO # Default item color BLANCO
                if baud_val == selected_baud_rate_in_popup: item_color = VERDE # Selected item VERDE
                
                baud_item_surf = font.render(str(baud_val), True, item_color)
                pantalla.blit(baud_item_surf, baud_item_surf.get_rect(centery=item_rect.centery, left=item_rect.left + 5))
                temp_y_list_item += dropdown_item_height
            current_y_offset_puerto = list_box_y + actual_list_height + 10
        else:
            current_y_offset_puerto = puerto_popup_select_baud_rect.bottom + 10

        # Status Message
        if puerto_popup_message:
            msg_surf = font.render(puerto_popup_message, True, BLANCO) # Message text BLANCO
            msg_rect = msg_surf.get_rect(centerx=puerto_popup_main_rect.centerx, top=current_y_offset_puerto)
            pantalla.blit(msg_surf, msg_rect)
            current_y_offset_puerto = msg_rect.bottom + 10
        
        # Buttons (Apply, Cancel) 
        button_width_puerto = 100 # Renamed to avoid conflict
        button_height_puerto = 30 # Renamed
        gap_between_buttons_puerto = 10 # Renamed
        
        puerto_popup_cancel_button_rect.width = button_width_puerto
        puerto_popup_cancel_button_rect.height = button_height_puerto
        puerto_popup_cancel_button_rect.bottom = puerto_popup_main_rect.bottom - popup_internal_padding_x 
        puerto_popup_cancel_button_rect.right = puerto_popup_main_rect.centerx - (gap_between_buttons_puerto / 2)
        pygame.draw.rect(pantalla, NEGRO, puerto_popup_cancel_button_rect) # Button BG NEGRO
        pygame.draw.rect(pantalla, VERDE, puerto_popup_cancel_button_rect, 1) # Button Border VERDE
        pantalla.blit(puerto_popup_cancel_text_surf, puerto_popup_cancel_text_surf.get_rect(center=puerto_popup_cancel_button_rect.center)) # Text BLANCO from re-render

        puerto_popup_apply_button_rect.width = button_width_puerto
        puerto_popup_apply_button_rect.height = button_height_puerto
        puerto_popup_apply_button_rect.bottom = puerto_popup_main_rect.bottom - popup_internal_padding_x 
        puerto_popup_apply_button_rect.left = puerto_popup_main_rect.centerx + (gap_between_buttons_puerto / 2)
        pygame.draw.rect(pantalla, NEGRO, puerto_popup_apply_button_rect) # Button BG NEGRO
        pygame.draw.rect(pantalla, VERDE, puerto_popup_apply_button_rect, 1) # Button Border VERDE
        pantalla.blit(puerto_popup_apply_text_surf, puerto_popup_apply_text_surf.get_rect(center=puerto_popup_apply_button_rect.center)) # Text BLANCO from re-render
    # --- End Draw "Puerto" Pop-up Window ---

    # --- Draw Main Menu Pop-up Panel ---
    if show_main_menu_popup:
        menu_panel_width = 300  # Fixed width
        menu_panel_margin_top = 10
        menu_panel_margin_right = 10
        menu_panel_internal_padding_top = 10 # Padding inside the panel, above title
        menu_panel_internal_padding_bottom = 15 # Padding inside the panel, below last item
        spacing_between_items = 8 # Consistent spacing for height calculation
        
        _menu_item_row_h = font_menu_item.get_linesize() + 4 # Adjusted item height for new font
        _base_dropdown_item_h = font_menu_item.get_linesize() + 2

        # --- Calculate Content Height ---
        _content_total_h = 0 
        # No title, start with top padding for the first item
        # _content_total_h += font_large.get_height() + 15 # title + its bottom padding (15 is from current_menu_y_offset = title_menu_rect.bottom + 15)
        
        _all_menu_items_config = [
            ("dropdown", "potencia_tx", "POTENCIA TX:", range(1, 11), None),
            ("dropdown", "long_impulso", "LONG IMPULSO:", range(1, 11), None),
            ("dropdown", "ciclo_tx", "CICLO TX:", range(1, 11), None),
            ("dropdown", "tvg_proximo", "TVG PROXIMO:", range(1, 11), None),
            ("dropdown", "tvg_lejano", "TVG LEJANO:", range(1, 11), None),
            ("dropdown", "cag", "CAG:", range(1, 11), None),
            ("dropdown", "cag_2", "2° CAG:", range(1, 11), None),
            ("dropdown", "limitar_ruido", "LIMITAR RUIDO:", range(1, 11), None),
            ("square", "curva_color", "CURVA COLOR:", [str(i) for i in range(1, 5)], "ROJO"),
            ("square", "respuesta_color", "RESPUESTA COLOR:", [str(i) for i in range(1, 5)], "ROJO"),
            ("dropdown", "anular_color", "ANULAR COLOR:", range(1, 11), None),
            ("dropdown", "promedio_eco", "PROMEDIO ECO:", range(1, 4), None),
            ("dropdown", "rechazo_interf", "RECHAZO INTERF:", range(1, 4), None),
            ("square", "angulo_haz_hor", "ANGULO HAZ HOR:", ["ANCHO", "ESTRECHO"], "ROJO"),
            ("square", "angulo_haz_ver", "ANGULO HAZ VER:", ["ANCHO", "ESTRECHO"], "ROJO"),
            ("square", "color_menu", "COLOR:", [str(i) for i in range(1, 5)], "VERDE_CLARO"),
            # BORRAR MARCAS items are action buttons, not value states here
            # ("dropdown", "nivel_alarma", "NIVEL ALARMA:", range(1,11), None), # REMOVED
            # ("square", "explor_auto", "EXPLOR AUTO:", ["ON", "OFF"], "VERDE_CLARO"), # REMOVED
            # ("dropdown", "sector_explor", "SECTOR EXPLOR:", ["±10°", "±20°", "±30°", "±60°", "±90°", "±170°"], None), # REMOVED
            # ("square", "inclin_auto", "INCLIN AUTO:", ["ON", "OFF"], "VERDE_CLARO"), # REMOVED
            # ("dropdown", "angulo_inclin", "ANGULO INCLIN:", ["±2-10°", "±2-20°", "±2-30°", "±2-40°", "±2-55°"], None), # REMOVED
            ("square", "transmision", "TRANSMISION:", ["ON", "OFF"], "VERDE_CLARO"),
            ("dropdown", "volumen_audio", "VOLUMEN AUDIO:", range(0, 11), None), # Changed range to 0-10
        ]
        
        for item_config_type, _, _, _, _ in _all_menu_items_config:
            _content_total_h += _menu_item_row_h + spacing_between_items
        
        # Subtract one spacing_between_items as the last item doesn't have spacing *after* it before bottom padding
        if len(_all_menu_items_config) > 0: # if there's any content
             _content_total_h -= spacing_between_items # remove last spacing if items exist


        calculated_panel_height = menu_panel_internal_padding_top + _content_total_h + menu_panel_internal_padding_bottom
        
        menu_panel_rect = pygame.Rect(
            dimensiones[0] - menu_panel_width - menu_panel_margin_right, # Right aligned
            menu_panel_margin_top,
            menu_panel_width,
            min(calculated_panel_height, dimensiones[1] - menu_panel_margin_top - 10) # Max height, 10px margin bottom
        )
        # Ensure minimum height if calculated is too small or negative due to screen constraints
        menu_panel_rect.height = max(menu_panel_rect.height, 50)

        # Use GRIS_MEDIO for menu panel background, BLANCO for border (consistent for now)
        pygame.draw.rect(pantalla, GRIS_MEDIO, menu_panel_rect) 
        pygame.draw.rect(pantalla, BLANCO, menu_panel_rect, 2)   

        # Title removed
        
      
        global_menu_panel_rect = menu_panel_rect 

        # --- Menu Content Drawing ---
        current_menu_y_offset = menu_panel_rect.top + menu_panel_internal_padding_top 
        menu_item_height = _menu_item_row_h 
        menu_padding_x = 10
        
        interactive_menu_item_rects = {}
        base_dropdown_item_height = _base_dropdown_item_h 
        color_palette_map = {"NEGRO": NEGRO, "BLANCO": BLANCO} 
        square_size_calc = int(font_menu_item.get_height() * 2.0) 
        color_palette_map_sq = {"NEGRO": NEGRO, "BLANCO": BLANCO, "ROJO": ROJO, "VERDE_CLARO": VERDE_CLARO}

        for item_type, key, label, item_source_or_labels, highlight_color_key in _all_menu_items_config:
            current_value = menu_options_values[key]
            occupied_h = 0

            if item_type == "dropdown":
                is_open = menu_dropdown_states[key]
                drawn_elements, occupied_h = draw_single_dropdown_option(
                    pantalla, key, label, current_value, is_open,
                    current_menu_y_offset, menu_panel_rect, font_menu_item, 
                    color_palette_map,
                    menu_item_height, 
                    base_dropdown_item_height, 
                    item_source_or_labels 
                )
                interactive_menu_item_rects[key] = drawn_elements
            elif item_type == "square":
                item_values = item_source_or_labels 
                drawn_sq_rects, occupied_h = draw_square_selector_option(
                    pantalla, key, label, current_value, item_source_or_labels,
                    current_menu_y_offset, menu_panel_rect, font_menu_item,
                    color_palette_map_sq, menu_item_height, square_size_calc, 
                    color_palette_map_sq[highlight_color_key]
                )
                interactive_menu_item_rects[key] = {'squares': drawn_sq_rects, 'values': item_values}
            
            current_menu_y_offset += occupied_h + spacing_between_items
        
        # --- ASIGNAR AJUSTE (Placeholder) - REMOVED ---
        # asignar_label_text = "ASIGNAR AJUSTE:"
        # asignar_label_surf = font_menu_item.render(asignar_label_text, True, BLANCO) 
        # asignar_label_rect = asignar_label_surf.get_rect(topleft=(menu_panel_rect.left + menu_padding_x, current_menu_y_offset))
        # pantalla.blit(asignar_label_surf, asignar_label_rect)
        # current_menu_y_offset += asignar_label_surf.get_height() + spacing_between_items

        # f_key_options = ["TECLA F1", "TECLA F2", "TECLA F3", "TECLA F4"] # REMOVED
        # for f_key_text in f_key_options:
            # f_key_surf = font_menu_item.render(f_key_text, True, GRIS_MEDIO) 
            # f_key_rect = f_key_surf.get_rect(topleft=(menu_panel_rect.left + menu_padding_x + 20, current_menu_y_offset))
            # pantalla.blit(f_key_surf, f_key_rect)
            # current_menu_y_offset += f_key_surf.get_height() + 5 


    # --- End Draw Main Menu Pop-up Panel ---

    # Avancemos y actualicemos la pantalla con lo que hemos dibujado.
    pygame.display.flip()

    # Limitamos a 60 fotogramas por segundo
    reloj.tick(60)

if serial_port_available and ser is not None:
    ser.close()
pygame.quit()




