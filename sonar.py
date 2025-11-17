# Importamos las bibliotecas pygame y math
import pygame
import math
import numpy as np # Importar NumPy
import os # Needed for path manipulation
import serial # Already present
import serial.tools.list_ports # For COM port detection
import functools
import operator
import json
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

def is_valid_nmea_checksum(sentence):
    """
    Validates the checksum of an NMEA sentence.
    Returns True if the checksum is valid or if the sentence has no checksum.
    Returns False if the checksum is present but invalid.
    """
    if '*' not in sentence:
        return True

    data_part, checksum_part = sentence.rsplit('*', 1)

    if not checksum_part or not all(c in '0123456789ABCDEFabcdef' for c in checksum_part):
        return False

    if data_part.startswith('$'):
        data_part = data_part[1:]

    calculated_checksum = functools.reduce(operator.xor, (ord(c) for c in data_part), 0)

    try:
        received_checksum = int(checksum_part, 16)
    except ValueError:
        return False

    return calculated_checksum == received_checksum

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

        # Validate the calculated decimal degrees
        if hemisphere in ['N', 'S']:  # It's a latitude
            if not -90.0 <= decimal_degrees <= 90.0:
                print(f"Invalid latitude calculated: {decimal_degrees}. Out of [-90, 90] range. Original: {nmea_val_str}{hemisphere}")
                return None
        elif hemisphere in ['E', 'W']:  # It's a longitude
            if not -180.0 <= decimal_degrees <= 180.0:
                print(f"Invalid longitude calculated: {decimal_degrees}. Out of [-180, 180] range. Original: {nmea_val_str}{hemisphere}")
                return None

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




# --- NUEVO: Clase para el Sistema de Menú ---
class MenuSystem:
    def __init__(self):
        self.active = False
        self.tabs = ['SONAR', 'SONDA', 'MARCAS', 'SISTEMA']
        self.active_tab = 'SISTEMA'

        # Fuentes
        self.font_tab = pygame.font.Font(None, 28)
        self.font_item_label = pygame.font.Font(None, 20)
        self.font_item_value = pygame.font.Font(None, 20)

        # Colores
        self.color_bg = (50, 50, 70)
        self.color_border = (150, 150, 180)
        self.color_text_light = (230, 230, 255)
        self.color_text_dim = (180, 180, 200)
        self.color_tab_active = (80, 80, 110)
        self.color_highlight = (144, 238, 144) # Verde claro
        self.color_focus_outline = (255, 255, 0) # Amarillo para el foco

        # Opciones del menú con valores por defecto
        self.options = self._get_default_options()

        self.focused_item_index = 0
        self.sistema_layout = self._create_sistema_layout()
        self.sonar_layout = self._create_sonar_layout()
        self.marcas_layout = self._create_marcas_layout()
        self.sonda_layout = self._create_sonda_layout()
        self.item_rects = {}
        self.puerto_com_dropdown_open = False
        self.com_ports_list = []
        self.scroll_offset = 0
        self.content_height = 0
        self.focus_on_tabs = False

    def _get_default_options(self):
        return {
            # Opciones de la pestaña SONAR
            'modo_presentac': 'COMBI-1',
            'potencia_tx': 8,
            'long_impulso': 8,
            'ciclo_tx': 10,
            'tvg_proximo': 6,
            'tvg_lejano': 7,
            'cag': 2,
            'cag_2': 1,
            'limitar_ruido': 3,
            'curva_color': 1,
            'respuesta_color': 1,
            'anular_color': 0,
            'promedio_eco': 1,
            'rechazo_interf': 1,
            'angulo_haz_hor': 'ANCHO',
            'angulo_haz_ver': 'ANCHO',
            'color': 1,
            'borrar_marcas': 'DERROTA',
            'nivel_alarma': 9,
            'explor_auto': 'ON',
            'sector_explor': '±10°',
            'inclin_auto': 'ON',
            'angulo_inclin': '±2-10°',
            'transmision': 'ON',
            'volumen_audio': 10,
            'asignar_ajuste': 'TECLA F1',
            'asignar_menu': None, # Placeholder for action

            # Opciones de la pestaña MARCAS
            'anillos_dist': '1/4R',
            'escala_demora': 'ON',
            'vector_corrnte': 'ON',
            'direc_corrnte': 'HACIA',
            'derrota_barco': '10R',
            'rumbo': '32PSCM',
            'rumbo_proa': '32PSCM',
            'datos_corrnte': '32PSCM',
            'evento_pesca': '32PSCM',
            'otras_marcas': '±180°',
            'datos_posicion': 'L/L',

            # Opciones de la pestaña SONDA
            'sonda_color': 1,
            'sonda_escala': 160,
            'sonda_desplazar_esc': 0,
            'sonda_rechz_interf': 'ON',
            'sonda_ganancia': 30,
            'sonda_filtro_parasit': 20,
            'sonda_avance_imagen': '2/1',
            'sonda_curva_color': 'LINEAL',
            'sonda_anular_color': 0,
            'sonda_ajuste_calado': 0,

            # Opciones de la pestaña SISTEMA
            'iluminacion': 5,
            'selec_present': 'TEMP',
            'ajuste_proa': 0,
            'subida_auto': 'OFF',
            'mensaje_veloc': 'ON',
            'puls_sinc_ext': 'OFF',
            'veloc_autoexpl': 'BAJA',
            'veloc_autoincl': 'BAJA',
            'unidad': 'METROS',
            'veloc_rumbo': 'DATO NAV',
            'pulso_log': 200,
            'puerto_com': 'Ninguno',
            'port_formato': 'NMEA',
            'port_baudios': 9600,
            'datos_nav': 'GPS',
            'escala_combi': 'DERECHA',
            'indi_subtexto': 'ON',
            'idioma': 'ESPANOL',
            'test': 'OFF',
            'aj_por_defecto': False
        }

    def _create_sistema_layout(self):
        return [
            {'label': 'ILUMINACION', 'key': 'iluminacion', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'SELEC PRESENT', 'key': 'selec_present', 'type': 'selector', 'values': ['TEMP', 'CORRNTE']},
            {'label': 'AJUSTE PROA', 'key': 'ajuste_proa', 'type': 'numeric_adjustable', 'range': (0, 359)},
            {'label': 'SUBIDA AUTO', 'key': 'subida_auto', 'type': 'selector', 'values': ['OFF', '5-16 kn']},
            {'label': 'MENSAJE VELOC', 'key': 'mensaje_veloc', 'type': 'selector', 'values': ['ON', 'OFF']},
            {'label': 'PULS SINC EXT', 'key': 'puls_sinc_ext', 'type': 'selector', 'values': ['OFF', 'ON']},
            {'label': 'VELOC AUTOEXPL', 'key': 'veloc_autoexpl', 'type': 'selector', 'values': ['BAJA', 'ALTA']},
            {'label': 'VELOC AUTOINCL', 'key': 'veloc_autoincl', 'type': 'selector', 'values': ['BAJA', 'ALTA']},
            {'label': 'UNIDAD', 'key': 'unidad', 'type': 'selector', 'values': ['METROS', 'PIES', 'BRAZAS', 'PA/BZS']},
            {'label': 'VELOC / RUMBO', 'key': 'veloc_rumbo', 'type': 'selector', 'values': ['LOG/GIRO', 'CORRNTE', 'DATO NAV', 'GIRO+NAV']},
            {'label': 'PULSO LOG', 'key': 'pulso_log', 'type': 'selector', 'values': [200, 400]},
            {'label': 'PUERTO COM', 'key': 'puerto_com', 'type': 'dropdown'},
            {'label': 'PORT FORMATO', 'key': 'port_formato', 'type': 'selector', 'values': ['NMEA', 'CIF']},
            {'label': 'PORT BAUDIOS', 'key': 'port_baudios', 'type': 'selector', 'values': [19200, 9600, 4800, 2400]},
            {'label': 'DATOS NAV', 'key': 'datos_nav', 'type': 'selector', 'values': ['GPS', 'LC', 'ESTIMA', 'TODOS']},
            {'label': 'ESCALA COMBI', 'key': 'escala_combi', 'type': 'selector', 'values': ['DERECHA', 'IZQUIERDA']},
            {'label': 'INDI SUBTEXTO', 'key': 'indi_subtexto', 'type': 'selector', 'values': ['OFF', 'ON']},
            {'label': 'IDIOMA', 'key': 'idioma', 'type': 'selector', 'values': ['ENGLISH', 'JAPANESE', 'ESPANOL', 'DANSK', 'NEDERLND', 'FRANCAIS', 'ITALIANO', 'KOREAN', 'NORSK']},
            {'label': 'TEST', 'key': 'test', 'type': 'selector', 'values': ['UNA VEZ', 'CONTINUO', 'PANEL', 'COLOR', 'PATTERN', 'SIO', 'ECO-1', 'ECO-2', 'ECO-3', 'ECO-4']},
            {'label': 'AJ POR DEFECTO', 'key': 'aj_por_defecto', 'type': 'action', 'button_text': 'EJECUTAR'},
        ]

    def _create_sonar_layout(self):
        return [
            {'label': 'MODO PRESENTAC', 'key': 'modo_presentac', 'type': 'selector', 'values': ['COMBI-1', 'NORMAL', 'COMBI-2']},
            {'label': 'POTENCIA TX', 'key': 'potencia_tx', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'LONG IMPULSO', 'key': 'long_impulso', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'CICLO TX', 'key': 'ciclo_tx', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'TVG PROXIMO', 'key': 'tvg_proximo', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'TVG LEJANO', 'key': 'tvg_lejano', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'CAG', 'key': 'cag', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': '2° CAG', 'key': 'cag_2', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'LIMITAR RUIDO', 'key': 'limitar_ruido', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'CURVA COLOR', 'key': 'curva_color', 'type': 'selector', 'values': [1, 2, 3, 4]},
            {'label': 'RSPUESTA COLOR', 'key': 'respuesta_color', 'type': 'selector', 'values': [1, 2, 3, 4]},
            {'label': 'ANULAR COLOR', 'key': 'anular_color', 'type': 'numeric_adjustable', 'range': (0, 10)},
            {'label': 'PROMEDIO ECO', 'key': 'promedio_eco', 'type': 'numeric_adjustable', 'range': (0, 3)},
            {'label': 'RECHAZ INTERF', 'key': 'rechazo_interf', 'type': 'numeric_adjustable', 'range': (0, 3)},
            {'label': 'ANGULO HAZ HOR', 'key': 'angulo_haz_hor', 'type': 'selector', 'values': ['ANCHO', 'ESTRECHO']},
            {'label': 'ANGULO HAZ VER', 'key': 'angulo_haz_ver', 'type': 'selector', 'values': ['ANCHO', 'ESTRECHO']},
            {'label': 'COLOR', 'key': 'color', 'type': 'selector', 'values': [1, 2, 3, 4]},
            {'label': 'BORRAR MARCAS', 'key': 'borrar_marcas', 'type': 'selector', 'values': ['DERROTA', 'BARCO', 'EVENTO', 'PESCA']},
            {'label': 'NIVEL ALARMA', 'key': 'nivel_alarma', 'type': 'numeric_adjustable', 'range': (1, 10)},
            {'label': 'EXPLOR AUTO', 'key': 'explor_auto', 'type': 'selector', 'values': ['ON', 'OFF']},
            {'label': 'SECTOR EXPLOR', 'key': 'sector_explor', 'type': 'selector', 'values': ['±10°', '±20°', '±40°', '±60°']},
            {'label': 'INCLIN AUTO', 'key': 'inclin_auto', 'type': 'selector', 'values': ['ON', 'OFF']},
            {'label': 'ANGULO INCLIN', 'key': 'angulo_inclin', 'type': 'selector', 'values': ['±2-10°', '±4-14°', '±6-20°', '±10-26°']},
            {'label': 'TRANSMISION', 'key': 'transmision', 'type': 'selector', 'values': ['ON', 'OFF']},
            {'label': 'VOLUMEN AUDIO', 'key': 'volumen_audio', 'type': 'numeric_adjustable', 'range': (0, 10)},
            {'label': 'ASIGNAR AJUSTE', 'key': 'asignar_ajuste', 'type': 'selector', 'values': ['TECLA F1', 'TECLA F2', 'TECLA F3', 'TECLA F4']},
            {'label': 'ASIGNAR MENU', 'key': 'asignar_menu', 'type': 'action', 'button_text': 'EJECUTAR'},
        ]

    def _create_marcas_layout(self):
        return [
            {'label': 'ANILLOS DIST', 'key': 'anillos_dist', 'type': 'selector', 'values': ['1/4R', '1/2R', 'OFF']},
            {'label': 'ESCALA DEMORA', 'key': 'escala_demora', 'type': 'selector', 'values': ['ON', 'OFF']},
            {'label': 'VECTOR CORRNTE', 'key': 'vector_corrnte', 'type': 'selector', 'values': ['ON', 'OFF']},
            {'label': 'DIREC CORRNTE', 'key': 'direc_corrnte', 'type': 'selector', 'values': ['HACIA', 'DESDE']},
            {'label': 'DERROTA BARCO', 'key': 'derrota_barco', 'type': 'selector', 'values': ['10R', '5R', 'OFF']},
            {'label': 'RUMBO', 'key': 'rumbo', 'type': 'selector', 'values': ['32PSCM', '360ºTRUE']},
            {'label': 'RUMBO PROA', 'key': 'rumbo_proa', 'type': 'selector', 'values': ['32PSCM', '360ºTRUE', 'OFF']},
            {'label': 'DATOS CORRNTE', 'key': 'datos_corrnte', 'type': 'selector', 'values': ['32PSCM', '360ºTRUE', '±180°', '360°']},
            {'label': 'EVENTO/PESCA', 'key': 'evento_pesca', 'type': 'selector', 'values': ['32PSCM', '360ºTRUE', '±180°', '360°']},
            {'label': 'OTRAS MARCAS', 'key': 'otras_marcas', 'type': 'selector', 'values': ['±180°', '360°']},
            {'label': 'DATOS POSICION', 'key': 'datos_posicion', 'type': 'selector', 'values': ['L/L', 'TD']},
        ]

    def _create_sonda_layout(self):
        return [
            {'label': 'COLOR', 'key': 'sonda_color', 'type': 'selector', 'values': [1, 2, 3, 4]},
            {'label': 'ESCALA', 'key': 'sonda_escala', 'type': 'numeric_adjustable', 'range': (10, 2000)},
            {'label': 'DESPLAZAR ESC.', 'key': 'sonda_desplazar_esc', 'type': 'numeric_adjustable', 'range': (0, 500)},
            {'label': 'RECHZ INTERF', 'key': 'sonda_rechz_interf', 'type': 'selector', 'values': ['ON', 'OFF']},
            {'label': 'GANANCIA', 'key': 'sonda_ganancia', 'type': 'numeric_adjustable', 'range': (0, 100)},
            {'label': 'FILTRO PARASIT', 'key': 'sonda_filtro_parasit', 'type': 'numeric_adjustable', 'range': (0, 100)},
            {'label': 'AVANCE IMAGEN', 'key': 'sonda_avance_imagen', 'type': 'selector', 'values': ['2/1', '1/1', '1/2', '1/4', '1/8']},
            {'label': 'CURVA COLOR', 'key': 'sonda_curva_color', 'type': 'selector', 'values': ['LINEAL', '1', '2', '3']},
            {'label': 'ANULAR COLOR', 'key': 'sonda_anular_color', 'type': 'numeric_adjustable', 'range': (0, 10)},
            {'label': 'AJUSTE CALADO', 'key': 'sonda_ajuste_calado', 'type': 'numeric_adjustable', 'range': (0, 200)},
        ]

    def toggle(self):
        self.active = not self.active
        if self.active:
            self.focused_item_index = 0
            self.com_ports_list = get_available_com_ports()

    def handle_event(self, event):
        if not self.active:
            if event.type == pygame.KEYDOWN and event.key == pygame.K_m:
                self.toggle()
            return

        if event.type == pygame.KEYDOWN:
            if event.key == pygame.K_m or event.key == pygame.K_ESCAPE:
                self.toggle()
                return

            if self.puerto_com_dropdown_open: # Si el dropdown está abierto, las teclas de navegación no hacen nada más
                return

            # Determinar el layout activo para la navegación
            if self.active_tab == 'SONAR':
                active_layout = self.sonar_layout
            elif self.active_tab == 'MARCAS':
                active_layout = self.marcas_layout
            elif self.active_tab == 'SONDA':
                active_layout = self.sonda_layout
            else: # SISTEMA
                active_layout = self.sistema_layout

            if self.focus_on_tabs:
                if event.key == pygame.K_h:
                    self.focus_on_tabs = False
                elif event.key == pygame.K_o:
                    current_tab_index = self.tabs.index(self.active_tab)
                    self.active_tab = self.tabs[(current_tab_index + 1) % len(self.tabs)]
                    self.focused_item_index = 0
                elif event.key == pygame.K_l:
                    current_tab_index = self.tabs.index(self.active_tab)
                    self.active_tab = self.tabs[(current_tab_index - 1) % len(self.tabs)]
                    self.focused_item_index = 0
            else: # Focus is on the content
                if event.key == pygame.K_y or event.key == pygame.K_UP:
                    if self.focused_item_index > 0:
                        self.focused_item_index -= 1
                    else: # We are at the top item, move focus to tabs
                        self.focus_on_tabs = True
                elif event.key == pygame.K_h or event.key == pygame.K_DOWN:
                    if self.focused_item_index < len(active_layout) - 1:
                        self.focused_item_index += 1

                # Handle value changes with O and L
                if self.focused_item_index < len(active_layout):
                    focused_item = active_layout[self.focused_item_index]
                    key = focused_item['key']

                    if focused_item['type'] in ['numeric_adjustable', 'numeric']:
                        min_val, max_val = focused_item['range']
                        if event.key == pygame.K_o:
                            self.options[key] = min(self.options[key] + 1, max_val)
                        elif event.key == pygame.K_l:
                            self.options[key] = max(self.options[key] - 1, min_val)

                    elif focused_item['type'] == 'selector':
                        valid_values = focused_item.get('values', [])
                        if valid_values:
                            current_value = self.options.get(key)
                            if current_value not in valid_values:
                                current_value = valid_values[0]
                            current_index = valid_values.index(current_value)

                            if event.key == pygame.K_o:
                                new_index = (current_index + 1) % len(valid_values)
                                self.options[key] = valid_values[new_index]
                            elif event.key == pygame.K_l:
                                new_index = (current_index - 1) % len(valid_values)
                                self.options[key] = valid_values[new_index]

                    # Post-change logic (e.g., for volume)
                    if focused_item['key'] == 'volumen_audio':
                        if sonar_ping_sound:
                            new_volume = self.options['volumen_audio'] / 10.0
                            sonar_ping_sound.set_volume(new_volume)

            # --- Auto-scroll logic ---
            # This is a simplified calculation of item y-position and height
            item_y = 0
            item_h = 0
            temp_y = 0
            value_start_x = self.main_panel_rect.left + 20 + 180
            for i, item in enumerate(active_layout):
                item_h = self.font_item_label.get_height()
                if item['type'] == 'selector':
                    row_y = 0
                    current_val_x = value_start_x
                    max_h_in_row = 0
                    for val in item['values']:
                        val_surf = self.font_item_value.render(str(val), True, self.color_text_light)
                        rect_w = val_surf.get_width() + 16
                        rect_h = val_surf.get_height() + 4
                        max_h_in_row = max(max_h_in_row, rect_h)
                        if current_val_x + rect_w > self.main_panel_rect.right - 20:
                            row_y += max_h_in_row + 5
                            current_val_x = value_start_x
                            max_h_in_row = rect_h
                        current_val_x += rect_w + 5
                    item_h = row_y + max_h_in_row

                if i == self.focused_item_index:
                    item_y = temp_y
                    break
                temp_y += item_h + 12

            visible_height = self.main_panel_rect.height - 120 # content_margin_y * 2
            if item_y < self.scroll_offset:
                self.scroll_offset = item_y
            elif item_y + item_h > self.scroll_offset + visible_height:
                self.scroll_offset = item_y + item_h - visible_height


        if event.type == pygame.MOUSEBUTTONDOWN:
            # Handle tab clicks
            for i, tab_name in enumerate(self.tabs):
                if hasattr(self, 'tab_rects') and i < len(self.tab_rects) and self.tab_rects[i].collidepoint(event.pos):
                    if self.active_tab != tab_name:
                        self.active_tab = tab_name
                        self.focused_item_index = 0
                    return None # Return None to signify event was handled

            # Handle clicks within the active tab content
            if self.active_tab == 'SONAR':
                active_layout = self.sonar_layout
            elif self.active_tab == 'MARCAS':
                active_layout = self.marcas_layout
            elif self.active_tab == 'SONDA':
                active_layout = self.sonda_layout
            else: # SISTEMA
                active_layout = self.sistema_layout

            # Handle dropdown box click first
            if 'puerto_com' in self.item_rects and 'main_box' in self.item_rects['puerto_com']:
                if self.item_rects['puerto_com']['main_box'].collidepoint(event.pos):
                    self.puerto_com_dropdown_open = not self.puerto_com_dropdown_open
                    self.focused_item_index = active_layout.index(next(item for item in active_layout if item['key'] == 'puerto_com'))
                    return None # Event handled
                elif self.puerto_com_dropdown_open:
                    for i, rect in enumerate(self.item_rects['puerto_com'].get('items', [])):
                        if rect.collidepoint(event.pos):
                            self.options['puerto_com'] = self.com_ports_list[i]
                            self.puerto_com_dropdown_open = False
                            return None # Event handled
                    # If click is outside dropdown items but dropdown is open, just close it
                    self.puerto_com_dropdown_open = False
                    return None # Event handled

            # If not a dropdown interaction, check other elements
            for idx, item_config in enumerate(active_layout):
                key = item_config['key']
                if key in self.item_rects:
                    if item_config['type'] == 'selector':
                        for i, rect in enumerate(self.item_rects[key]):
                            if rect.collidepoint(event.pos):
                                self.options[key] = item_config['values'][i]
                                self.focused_item_index = idx
                                return None # Event handled
                    elif item_config['type'] == 'action':
                        if self.item_rects[key][0].collidepoint(event.pos):
                            self.focused_item_index = idx
                            if key == 'aj_por_defecto':
                                self.options = self._get_default_options()
                                print("INFO: Configuración restaurada a los valores por defecto.")
                                return None
                            elif key == 'aj_por_defecto':
                                self.options = self._get_default_options()
                                print("INFO: Configuración restaurada a los valores por defecto.")
                            else:
                                print(f"INFO: Botón de acción '{key}' presionado (sin acción implementada).")
                            self.focused_item_index = idx
                            return None

            # If click is outside the main panel, close the menu
            if hasattr(self, 'main_panel_rect') and not self.main_panel_rect.collidepoint(event.pos):
                self.toggle()
                return None # Event handled

    def draw(self, surface):
        if not self.active: return

        self.main_panel_rect = pygame.Rect(0, 0, 550, surface.get_height() - 100)
        self.main_panel_rect.center = surface.get_rect().center
        pygame.draw.rect(surface, self.color_bg, self.main_panel_rect, border_radius=10)
        pygame.draw.rect(surface, self.color_border, self.main_panel_rect, 2, border_radius=10)

        self.tab_rects = []
        tab_y = self.main_panel_rect.top
        current_x = self.main_panel_rect.left
        for tab_name in self.tabs:
            text_surf = self.font_tab.render(tab_name, True, self.color_text_light)
            padding = 15
            tab_rect = pygame.Rect(current_x, tab_y, text_surf.get_width() + 2 * padding, 40)
            self.tab_rects.append(tab_rect)

            if self.active_tab == tab_name:
                pygame.draw.rect(surface, self.color_tab_active, tab_rect, border_top_left_radius=10, border_top_right_radius=10)
                if self.focus_on_tabs:
                    pygame.draw.rect(surface, self.color_focus_outline, tab_rect.inflate(2,2), 2, border_top_left_radius=10, border_top_right_radius=10)

            surface.blit(text_surf, text_surf.get_rect(center=tab_rect.center))
            current_x += tab_rect.width

        if self.active_tab == 'SISTEMA':
            self._draw_tab(surface, self.sistema_layout)
        elif self.active_tab == 'SONAR':
            self._draw_tab(surface, self.sonar_layout)
        elif self.active_tab == 'SONDA':
            self._draw_tab(surface, self.sonda_layout)
        elif self.active_tab == 'MARCAS':
            self._draw_tab(surface, self.marcas_layout)
        else:
            placeholder_text = self.font_tab.render(f"Pestaña {self.active_tab} en construcción.", True, self.color_text_dim)
            surface.blit(placeholder_text, placeholder_text.get_rect(center=self.main_panel_rect.center))

    def _draw_tab(self, surface, layout):
        # Define content area within the main panel
        content_margin_x = 20
        content_margin_y = 60
        content_area_rect = self.main_panel_rect.inflate(-content_margin_x*2, -content_margin_y*2)
        content_area_rect.top = self.main_panel_rect.top + content_margin_y

        # --- 1. Pre-calculate total content height (Dry Run) ---
        total_content_height = 0
        value_start_x_dry_run = content_area_rect.left + 180 # Relative to screen, for width calculation
        for item in layout:
            item_height = self.font_item_label.get_height()
            if item['type'] == 'selector':
                row_y = 0
                current_val_x = value_start_x_dry_run
                max_h_in_row = 0
                for val in item['values']:
                    val_text = str(val)
                    val_surf = self.font_item_value.render(val_text, True, self.color_text_light)
                    rect_w = val_surf.get_width() + 16 # padding
                    rect_h = val_surf.get_height() + 4
                    max_h_in_row = max(max_h_in_row, rect_h)
                    if current_val_x + rect_w > self.main_panel_rect.right - content_margin_x:
                        row_y += max_h_in_row + 5
                        current_val_x = value_start_x_dry_run
                        max_h_in_row = rect_h
                    current_val_x += rect_w + 5
                item_height = row_y + max_h_in_row
            total_content_height += item_height + 12
        self.content_height = total_content_height

        # --- 2. Handle Scrolling Logic ---
        visible_height = content_area_rect.height
        has_scrollbar = self.content_height > visible_height

        # Clamp scroll_offset
        if has_scrollbar:
            max_scroll = self.content_height - visible_height
            self.scroll_offset = max(0, min(self.scroll_offset, max_scroll))
        else:
            self.scroll_offset = 0

        # --- 3. Create a subsurface for clipping and scrolling ---
        content_surface = pygame.Surface(content_area_rect.size, pygame.SRCALPHA)
        content_surface.fill((0,0,0,0)) # Transparent background

        # --- 4. Draw all items onto the subsurface with scroll offset ---
        start_x = 0 # Relative to content_surface
        start_y = 0 # Relative to content_surface
        value_start_x = start_x + 180
        current_y = start_y - self.scroll_offset
        self.item_rects.clear()

        for i, item in enumerate(layout):
            item_start_y = current_y
            item_content_height = self.font_item_label.get_height()
            is_focused = (i == self.focused_item_index)

            # --- Draw Label ---
            label_surf = self.font_item_label.render(item['label'], True, self.color_text_light)
            content_surface.blit(label_surf, (start_x, current_y))

            # --- Draw Controls ---
            if item['type'] in ['numeric_adjustable']:
                value = self.options[item['key']]
                if item['key'] in ['sonda_ganancia', 'sonda_filtro_parasit', 'sonda_ajuste_calado']:
                    value_text = f"{value / 10.0:.1f}"
                else:
                    value_text = f"{value}"

                if item['key'] == 'ajuste_proa': value_text += "°"
                if item['key'] == 'sonda_ajuste_calado': value_text += " (m)"

                value_surf = self.font_item_value.render(value_text, True, self.color_text_light)
                content_surface.blit(value_surf, (value_start_x, current_y))
            elif item['type'] == 'selector':
                self.item_rects[item['key']] = []
                current_val_x = value_start_x
                row_y = current_y
                max_h_in_row = 0
                for val in item['values']:
                    val_text = str(val)
                    val_surf = self.font_item_value.render(val_text, True, self.color_text_light)
                    padding = 8
                    rect_w = val_surf.get_width() + 2 * padding
                    rect_h = val_surf.get_height() + 4
                    max_h_in_row = max(max_h_in_row, rect_h)
                    if current_val_x + rect_w > content_area_rect.width:
                        row_y += max_h_in_row + 5
                        current_val_x = value_start_x
                        max_h_in_row = rect_h
                    rect = pygame.Rect(current_val_x, row_y - 2, rect_w, rect_h)
                    try:
                        list_item_type = type(val)
                        option_val = list_item_type(self.options[item['key']])
                        if option_val == val:
                            pygame.draw.rect(content_surface, self.color_highlight, rect, border_radius=4)
                    except (ValueError, KeyError): pass
                    content_surface.blit(val_surf, val_surf.get_rect(center=rect.center))
                    self.item_rects[item['key']].append(rect.move(content_area_rect.topleft)) # Store screen-space rect
                    current_val_x += rect.width + 5
                item_content_height = (row_y + max_h_in_row) - current_y
            elif item['type'] == 'action':
                 button_text = item['button_text']
                 button_surf = self.font_item_value.render(button_text, True, self.color_text_light)
                 padding = 10
                 rect = pygame.Rect(value_start_x, current_y - 2, button_surf.get_width() + 2 * padding, button_surf.get_height() + 4)
                 pygame.draw.rect(content_surface, (80, 80, 100), rect, border_radius=4)
                 content_surface.blit(button_surf, button_surf.get_rect(center=rect.center))
                 self.item_rects[item['key']] = [rect.move(content_area_rect.topleft)]
                 item_content_height = rect.height

            # --- Draw Focus Outline ---
            if is_focused and not self.focus_on_tabs and not self.puerto_com_dropdown_open:
                focus_rect = pygame.Rect(start_x - 5, item_start_y - 3, content_area_rect.width - 10, item_content_height + 6)
                pygame.draw.rect(content_surface, self.color_focus_outline, focus_rect, 1, border_radius=4)

            current_y += item_content_height + 12

        # --- 5. Blit the clipped content surface to the screen ---
        surface.blit(content_surface, content_area_rect.topleft)

        # --- 6. Draw Scrollbar if needed ---
        if has_scrollbar:
            scrollbar_area = pygame.Rect(self.main_panel_rect.right - 15, self.main_panel_rect.top + content_margin_y, 10, visible_height)
            pygame.draw.rect(surface, (40,40,60), scrollbar_area) # Scrollbar track

            handle_height = max(20, visible_height * (visible_height / self.content_height))
            handle_y = scrollbar_area.top + (self.scroll_offset / self.content_height) * visible_height
            scrollbar_handle = pygame.Rect(scrollbar_area.left, handle_y, 10, handle_height)
            pygame.draw.rect(surface, (180, 180, 200), scrollbar_handle, border_radius=5)

        # --- 7. Draw Dropdown (must be drawn on main surface to overlay everything) ---
        if self.puerto_com_dropdown_open:
            focused_item = layout[self.focused_item_index]
            if focused_item['key'] == 'puerto_com':
                # We need the on-screen rect, which we calculate from its relative position
                main_box_screen_rect = self.item_rects['puerto_com'][0].move(-self.scroll_offset, 0)
                dropdown_y = main_box_screen_rect.bottom + 2
                list_rect = pygame.Rect(main_box_screen_rect.left, dropdown_y, main_box_screen_rect.width, len(self.com_ports_list) * 22)
                pygame.draw.rect(surface, self.color_bg, list_rect)
                pygame.draw.rect(surface, self.color_border, list_rect, 1)
                for i, com_port in enumerate(self.com_ports_list):
                    item_rect = pygame.Rect(list_rect.left, dropdown_y, list_rect.width, 22)
                    item_surf = self.font_item_value.render(com_port, True, self.color_text_light)
                    surface.blit(item_surf, item_surf.get_rect(centery=item_rect.centery, left=item_rect.left + 5))
                    self.item_rects['puerto_com'].append(item_rect) # This part is tricky, rects are now dynamic
                    dropdown_y += item_rect.height

# --- Persistencia de Configuración (save/load) ---
CONFIG_FILE = "config.json"

def save_settings():
    global puerto, baudios, current_gain, current_range_index, current_tilt_angle, menu
    if 'menu' in globals() and isinstance(menu, MenuSystem):
        # Sincronizar variables principales con el menú antes de guardar
        puerto = menu.options.get('puerto_com', None)
        baudios = menu.options.get('port_baudios', 9600)

        settings_to_save = {
            'gain': current_gain,
            'range_index': current_range_index,
            'tilt_angle': current_tilt_angle,
            'menu_options': menu.options
        }
        try:
            with open(CONFIG_FILE, 'w') as f:
                json.dump(settings_to_save, f, indent=4)
        except IOError as e:
            print(f"Error al guardar configuración: {e}")

def load_settings():
    global puerto, baudios, current_gain, current_range_index, current_tilt_angle, menu, current_unit
    try:
        if os.path.exists(CONFIG_FILE):
            with open(CONFIG_FILE, 'r') as f:
                loaded_settings = json.load(f)

            current_gain = loaded_settings.get('gain', current_gain)
            current_range_index = loaded_settings.get('range_index', current_range_index)
            current_tilt_angle = loaded_settings.get('tilt_angle', current_tilt_angle)

            if 'menu' in globals() and isinstance(menu, MenuSystem) and 'menu_options' in loaded_settings:
                menu.options.update(loaded_settings['menu_options'])
                # Sincronizar estado principal con las opciones cargadas
                puerto = menu.options.get('puerto_com', None)
                baudios = menu.options.get('port_baudios', 9600)
                current_unit = menu.options.get('unidad', 'METROS').upper()
    except (IOError, json.JSONDecodeError) as e:
        print(f"Advertencia al cargar configuración: {e}")


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
        "DATA_PANEL_BG": (30, 60, 30),      # Un verde aún más oscuro para el panel
        "DATA_PANEL_BORDER": (107, 142, 35),
        "BUTTON_BG": (30, 60, 30),
        "BUTTON_BORDER": (107, 142, 35),
        "CURSOR_CROSS": (188, 238, 104),
        "RANGE_RINGS": (107, 142, 35),      # Verde oliva para los anillos
        "SHIP_TRACK": (152, 251, 152),      # Verde pálido para la estela
        "CENTER_ICON": (188, 238, 104),
        "MENU_ITEM_HIGHLIGHT": DEFAULT_VERDE_CLARO, # Mantener este por ahora o definir uno específico
        "MENU_ITEM_RED_HIGHLIGHT": DEFAULT_ROJO, # Mantener este por ahora o definir uno específico
    },
    2: { # Verde Claro
        "BACKGROUND": (144, 238, 144),      # Verde claro (LightGreen)
        "PRIMARY_TEXT": (0, 100, 0),        # Verde oscuro para texto
        "ACCENT_ELEMENT": (34, 139, 34),   # Verde bosque
        "SWEEP": (60, 179, 113),            # Verde mar medio
        "TARGET_BASE": (0, 100, 0),
        "TARGET_HOVER": (255, 0, 0),        # Rojo estándar para hover
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
        "BACKGROUND": (173, 216, 230),      # Celeste (LightBlue)
        "PRIMARY_TEXT": (0, 0, 128),        # Azul marino para texto
        "ACCENT_ELEMENT": (70, 130, 180),  # Azul acero
        "SWEEP": (135, 206, 250),          # Azul cielo claro
        "TARGET_BASE": (0, 0, 128),
        "TARGET_HOVER": (255, 0, 0),        # Rojo estándar
        "COMPASS_ROSE": (0, 0, 128),
        "DATA_PANEL_BG": (220, 240, 255),  # Azul muy pálido
        "DATA_PANEL_BORDER": (70, 130, 180),
        "BUTTON_BG": (220, 240, 255),
        "BUTTON_BORDER": (70, 130, 180),
        "CURSOR_CROSS": (0, 0, 128),
        "RANGE_RINGS": (70, 130, 180),      # Azul acero para los anillos
        "SHIP_TRACK": (135, 206, 250),      # Azul cielo claro para la estela
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
VERDE = DEFAULT_VERDE    # Usado para bordes de panel, etc.
ROJO = DEFAULT_ROJO      # Para hover de targets, etc.
AZUL = DEFAULT_AZUL      # Para fondo principal.
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
RANGE_PRESETS_METERS = [50, 85, 100, 150, 200, 250, 300, 350, 400, 450, 500, 600, 800, 1000, 1200, 1600]
RANGE_PRESETS_BRAZAS = [50, 75, 100, 125, 150, 175, 200, 225, 250, 300, 400, 500, 600, 800]
RANGE_PRESETS_PIES = [300, 450, 600, 750, 900, 1050, 1200, 1350, 1500, 1800, 2400, 3000, 3600, 4800]

current_unit = "METERS"
range_presets_map = {
    "METROS": RANGE_PRESETS_METERS,
    "BRAZAS": RANGE_PRESETS_BRAZAS,
    "PIES": RANGE_PRESETS_PIES,
    "PA/BZS": RANGE_PRESETS_BRAZAS
}
range_display_suffix_map = { # Units removed from R value displays
    "METROS": "",
    "BRAZAS": "",
    "PIES": "",
    "PA/BZS": ""
}
sonar_rose_unit_text_map = {
    "METROS": "(m)",
    "BRAZAS": "(fm)",
    "PIES": "(ft)",
    "PA/BZS": "(fm)"
}

# Initialize current_range_index (Moved up)
try:
    default_range_value_meters = 300
    current_range_index = RANGE_PRESETS_METERS.index(default_range_value_meters)
except ValueError:
    current_range_index = 4

# Font initialization
font = pygame.font.Font(None, 24) # Using a default system font
font_large = pygame.font.SysFont("segoeuisymbol", 30)
font_very_large = pygame.font.Font(None, 48)
font_data_medium = pygame.font.Font(None, 36)
font_size_50 = pygame.font.Font(None, 50)
font_size_54 = pygame.font.Font(None, 54)
font_menu_item = pygame.font.Font(None, 20) # New font for menu items

# --- OLD UI ELEMENT DEFINITIONS REMOVED ---
# This section previously contained variables for the old button-based
# menu popups (show_puerto_popup, menu_options_values, etc.).
# They have been removed to make way for the new MenuSystem class.
active_sonar_rose_unit_surface = None # Will be rendered in main loop or when colors change
# --- END REMOVED UI ELEMENT DEFINITIONS ---

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
# The global `current_volume` has been removed.
# The volume is now managed via `menu.options['volumen_audio']`.
# --- End Audio Volume ---

# --- Load Sonar Sound ---
# This is loaded after the menu is initialized, so we can set the initial volume.
sonar_ping_sound = None
# --- End Load Sonar Sound ---

# Text string definitions (already added in previous step, kept for completeness)
texto_latitud = "LAT / LON "
texto_longitud = "VELOC DEL BARCO" # This is "Ship Speed" label
texto_velocidad = "RUMBO DEL BARCO" # This is "Ship Course" label
#texto_dato_nuevo_1 = "HORA / FECHA"
# texto_dato_nuevo_2 = "RATE OF TURN" # ROT text definition removed
#texto_dato_nuevo_3 = "PITCH / ROLL"


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
triangle_markers = [] # List for independent triangle markers
TARGET_TYPE_RHOMBUS = "rhombus"
TARGET_TYPE_X = "x"
TARGET_TYPE_TRIANGLE = "triangle"
# All markers will be initially white. Hovering will make them red.
# COLOR_TARGET_BASE = BLANCO # Base color for all markers - Will use current_colors["TARGET_BASE"]
# COLOR_TARGET_HOVER = ROJO    # Color when hovered or selected - Will use current_colors["TARGET_HOVER"]

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
    "hovered_marker_index": None, # To store the index of the marker being hovered
    "hovered_marker_list": None, # To store which list ('target' or 'triangle') is hovered
    "cursor_lat_str": "N/A",
    "cursor_lon_str": "N/A"
}

# --- Key Event Handling Function ---
def handle_key_events(event_key, circle_center_x_param, circle_center_y_param, display_radius_pixels_param, s_max_current_range_param):
    global current_range_index, current_tilt_angle, show_tilt_temporarily, tilt_display_timer, \
           show_range_temporarily, range_display_timer, \
           current_gain, show_gain_temporarily, gain_display_timer, target_markers, \
           current_ship_lat_deg, current_ship_lon_deg, current_ship_heading, current_unit, \
           current_volume, menu_options_values, sonar_ping_sound, triangle_markers, effective_heading
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
    elif event_key == pygame.K_a:
        if ui_state["show_plus_cursor"]:
            mouse_cursor_x, mouse_cursor_y = ui_state["mouse_cursor_pos"]
            current_time = pygame.time.get_ticks()

            new_marker = {
                'mode': None, # Will be 'geo' or 'screen'
                'geo_pos': None, # For 'geo' mode
                'initial_screen_pos': None, # For 'screen' mode: (abs_x, abs_y) at creation, less critical now
                'initial_distance_meters': None, # For 'screen' mode: conceptual distance from center, in METERS
                'original_angle_rad': None, # For 'screen' mode: standard math angle (atan2(dy,dx)) from center
                'screen_bearing_rad': None, # For 'screen' mode: visual angle from screen up (atan2(dx,-dy)) for NMEA conversion
                'type': TARGET_TYPE_TRIANGLE,
                'timestamp': current_time,
                'current_screen_pos': None,
                'is_hovered': False
            }

            if current_ship_lat_deg is not None and current_ship_lon_deg is not None:
                new_marker['mode'] = 'geo'
                pixel_dist_from_center = math.sqrt((mouse_cursor_x - circle_center_x_param)**2 + (mouse_cursor_y - circle_center_y_param)**2)
                distance_m_from_ship = (pixel_dist_from_center / display_radius_pixels_param) * s_max_current_range_param if display_radius_pixels_param > 0 else 0
                if current_unit == "BRAZAS":
                    distance_m_from_ship *= 1.8288

                dx = mouse_cursor_x - circle_center_x_param
                dy = mouse_cursor_y - circle_center_y_param
                visual_angle_rad = math.atan2(dx, -dy)
                visual_bearing_deg = (math.degrees(visual_angle_rad) + 360) % 360
                true_bearing_deg = (visual_bearing_deg + effective_heading) % 360

                start_point = Point(latitude=current_ship_lat_deg, longitude=current_ship_lon_deg)
                if distance_m_from_ship < 0: distance_m_from_ship = 0
                destination = geodesic(meters=distance_m_from_ship).destination(point=start_point, bearing=true_bearing_deg)
                new_marker['geo_pos'] = {'lat': destination.latitude, 'lon': destination.longitude}
            else:
                new_marker['mode'] = 'screen'
                new_marker['initial_screen_pos'] = (mouse_cursor_x, mouse_cursor_y)
                new_marker['initial_S_max'] = s_max_current_range_param
                new_marker['initial_display_radius_pixels'] = display_radius_pixels_param

                pixel_dist_from_center = math.sqrt((mouse_cursor_x - circle_center_x_param)**2 + (mouse_cursor_y - circle_center_y_param)**2)
                if display_radius_pixels_param > 0:
                    conceptual_distance_in_current_units = (pixel_dist_from_center / display_radius_pixels_param) * s_max_current_range_param
                else:
                    conceptual_distance_in_current_units = 0

                if current_unit == "BRAZAS":
                    new_marker['initial_distance_meters'] = conceptual_distance_in_current_units * 1.8288
                else:
                    new_marker['initial_distance_meters'] = conceptual_distance_in_current_units

                dx_orig = mouse_cursor_x - circle_center_x_param
                dy_orig = mouse_cursor_y - circle_center_y_param
                new_marker['original_angle_rad'] = math.atan2(dy_orig, dx_orig)
                new_marker['screen_bearing_rad'] = math.atan2(dx_orig, -dy_orig)


                new_marker['geo_pos'] = None

            triangle_markers.append(new_marker)

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
                true_bearing_deg = (visual_bearing_deg + effective_heading) % 360

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
        if ui_state['hovered_marker_index'] is not None:
            if ui_state['hovered_marker_list'] == 'target' and ui_state['hovered_marker_index'] < len(target_markers):
                target_markers.pop(ui_state['hovered_marker_index'])

                # Re-evaluate T0, T1, T2 shapes after deletion
                if len(target_markers) >= 1:
                    target_markers[-1]['type'] = TARGET_TYPE_RHOMBUS
                if len(target_markers) >= 2:
                    target_markers[-2]['type'] = TARGET_TYPE_RHOMBUS
                if len(target_markers) >= 3:
                    for i in range(len(target_markers) - 2):
                        target_markers[i]['type'] = TARGET_TYPE_X
                if len(target_markers) == 1:
                    target_markers[0]['type'] = TARGET_TYPE_RHOMBUS
                elif len(target_markers) == 2:
                    target_markers[0]['type'] = TARGET_TYPE_RHOMBUS
                    target_markers[1]['type'] = TARGET_TYPE_RHOMBUS

            elif ui_state['hovered_marker_list'] == 'triangle' and ui_state['hovered_marker_index'] < len(triangle_markers):
                triangle_markers.pop(ui_state['hovered_marker_index'])

            ui_state['hovered_marker_index'] = None
            ui_state['hovered_marker_list'] = None


# --- Función para dibujar el eco de un cardumen ---
def dibujar_eco_cardumen(surface, center_x, center_y, angulo_barrido_rad, distancia_cardumen, tamano_cardumen, potencia_tx, display_radius_pixels):
    """
    Dibuja un círculo simple que representa el eco de un cardumen.
    La intensidad del eco depende de la potencia de transmisión (potencia_tx) y la distancia.
    """
    # El ángulo del cardumen debe ser relativo al barrido para que aparezca "pintado" por el haz.
    angulo_cardumen_rad = angulo_barrido_rad

    # Calcular la posición del cardumen
    cardumen_x = center_x + distancia_cardumen * math.sin(angulo_cardumen_rad)
    cardumen_y = center_y - distancia_cardumen * math.cos(angulo_cardumen_rad)

    # --- Algoritmo de intensidad del eco ---
    if display_radius_pixels <= 0:
        intensidad_eco = 0
    else:
        # Normalizar la distancia del cardumen a un ratio de 0.0 a 1.0 del radio total
        distancia_ratio = min(1.0, distancia_cardumen / display_radius_pixels)

        # Calcular la intensidad que tendría el eco en el rango máximo (borde de la pantalla)
        # Se mapea potencia_tx (rango 1-10) a una intensidad (rango 0-255)
        # Si potencia es 1, intensidad en el borde es 0. Si es 10, es 255.
        if potencia_tx <= 1:
            intensity_at_max_range = 0
        else:
            intensity_at_max_range = 255 * ((potencia_tx - 1) / 9.0)

        # La intensidad final es una interpolación lineal entre 255 (en el centro)
        # y `intensity_at_max_range` (en el borde).
        final_intensity = 255 * (1 - distancia_ratio) + intensity_at_max_range * distancia_ratio
        
        # Asegurarse de que el valor está en el rango 0-255
        intensidad_eco = max(0, min(255, int(final_intensity)))

    # Si la intensidad es muy baja, no dibujamos nada para optimizar.
    if intensidad_eco < 5:
        return

    color_eco = (255, 165, 0, intensidad_eco) # Naranja con alfa

    # Dibujar el eco del cardumen
    # Usamos una superficie temporal para aplicar el alfa correctamente.
    eco_surface = pygame.Surface((tamano_cardumen * 2, tamano_cardumen * 2), pygame.SRCALPHA)
    pygame.draw.circle(eco_surface, color_eco, (tamano_cardumen, tamano_cardumen), tamano_cardumen)
    surface.blit(eco_surface, (cardumen_x - tamano_cardumen, cardumen_y - tamano_cardumen))

# Inicia el bucle de eventos
terminado = False
angulo_barrido_rad = 0
menu = MenuSystem()
# Bucle principal del juego
while not terminado:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            terminado = True
        elif event.type == pygame.VIDEORESIZE:
            dimensiones = event.size
            screen = pygame.display.set_mode(dimensiones, pygame.RESIZABLE)
            pantalla = screen
        elif event.type == pygame.KEYDOWN:
            handle_key_events(event.key, circle_center_x, circle_center_y, display_radius_pixels, s_max_current_range)
        
        # Pasar eventos al sistema de menú
        menu.handle_event(event)


    # Lógica de reconexión del puerto serie
    current_time = pygame.time.get_ticks()
    if not serial_port_available and current_time - last_reconnect_attempt_time > RECONNECT_INTERVAL_MS:
        last_reconnect_attempt_time = current_time
        if puerto and puerto != 'Ninguno':
            try:
                ser = serial.Serial(puerto, baudios, timeout=1)
                serial_port_available = True
                print(f"Éxito al reconectar con {puerto}")
            except serial.SerialException as e:
                print(f"Fallo al intentar reconectar con {puerto}: {e}")
                serial_port_available = False

    # Actualizar colores si ha cambiado en el menú
    new_color_scheme_idx = menu.options.get('color', 3)
    if new_color_scheme_idx != active_color_scheme_idx:
        active_color_scheme_idx = new_color_scheme_idx
        current_colors = color_schemes.get(active_color_scheme_idx, color_schemes[3])

    # Sincronizar unidad desde el menú
    current_unit = menu.options.get('unidad', 'METROS').upper()


    # Leer y procesar datos del puerto serie
    if serial_port_available and ser and ser.is_open:
        try:
            line = ser.readline().decode('ascii', errors='replace').strip()
            if line and is_valid_nmea_checksum(line):
                # Existing sentence parsing logic...
                if line.startswith('$GPGLL') or line.startswith('$GNGLL'):
                    parse_gll(line)
                elif line.startswith('$GPGGA') or line.startswith('$GNGGA'):
                    parse_gga(line)
                elif line.startswith('$GPRMC') or line.startswith('$GNRMC'):
                    parse_rmc(line)
                elif line.startswith('$GPVTG') or line.startswith('$GNVTG'):
                    parse_vtg(line)
                elif line.startswith('$HEHDT'):
                    parse_hdt(line)
                elif line.startswith('$HCHDG'):
                    parse_hdg(line)
                elif line.startswith('$GPZDA') or line.startswith('$GNZDA'):
                    parse_zda(line)
                elif line.startswith('$PFEC,GPatt'):
                    parse_fec_gpatt(line)

        except serial.SerialException as e:
            print(f"Error de puerto serie: {e}")
            if ser: ser.close()
            serial_port_available = False
    
    # Manejar cambios de estado del puerto serie
    if prev_serial_port_available and not serial_port_available:
        # Si se acaba de desconectar, resetear datos a N/A
        # latitude_str = longitude_str = speed_str = heading_str = "N/A"
        # current_ship_lat_deg = current_ship_lon_deg = None
        pass
    
    prev_serial_port_available = serial_port_available


    # Llenamos el fondo de negro
    pantalla.fill(current_colors["BACKGROUND"])

    # --- Cálculos de Geometría del Sonar ---
    # Esto ahora se calcula en cada fotograma para manejar el redimensionamiento
    ancho_pantalla, alto_pantalla = pantalla.get_size()
    # Dejar espacio para el panel de datos a la derecha
    ancho_disponible_sonar = ancho_pantalla - unified_data_box_dims[2] - 30 # 30 para márgenes
    alto_disponible_sonar = alto_pantalla - 20 # 20 para márgenes
    
    # El radio del círculo del sonar se basa en la dimensión más pequeña
    display_radius_pixels = min(ancho_disponible_sonar, alto_disponible_sonar) / 2
    
    # Centrar el sonar en el área disponible
    circle_center_x = (ancho_disponible_sonar / 2) + 10
    circle_center_y = (alto_disponible_sonar / 2) + 10

    # Actualizar la posición de la caja de datos unificada
    unified_data_box_dims[0] = circle_center_x * 2 + 10
    unified_data_box_dims[1] = 10
    unified_data_box_dims[2] = ancho_pantalla - (circle_center_x * 2 + 20)
    unified_data_box_dims[3] = alto_pantalla - 20

    # --- Lógica del Barrido del Sonar ---
    # Obtener el rango máximo actual de los presets basado en la unidad
    s_max_current_range = range_presets_map[current_unit][current_range_index]

    # Calcular la duración de un barrido completo (ping-echo-return) en segundos
    # Tiempo = 2 * Rango / Velocidad del Sonido
    # Multiplicamos por 2 porque el radio del barrido se reinicia 2 veces por ciclo completo de `angulo_barrido_rad`
    # Esto es una simplificación; en un sonar real, el ciclo de TX es clave.
    # Usamos el valor del menú 'ciclo_tx' para influir en la velocidad.
    # Un valor más alto en el menú (10) debería resultar en un barrido más rápido (menos segundos).
    # Mapeamos el rango 1-10 a, por ejemplo, 10-1 segundos.
    tx_cycle_value = menu.options.get('ciclo_tx', 10)
    sweep_duration_seconds = 11 - tx_cycle_value # Mapeo simple: 10 -> 1s, 1 -> 10s
    
    # Calcular cuánto debe avanzar el ángulo en este fotograma
    # Avance = (2 * PI) / (FPS * Duración)
    fps = reloj.get_fps()
    if fps > 0:
        angle_increment = (2 * PI) / (fps * sweep_duration_seconds)
    else:
        angle_increment = 0.01 # Fallback
    
    angulo_barrido_rad = (angulo_barrido_rad + angle_increment) % (2 * PI)

    # El radio del barrido visual debe corresponder al tiempo de viaje del sonido
    # En esta simulación, simplemente lo hacemos coincidir con el ángulo para el efecto visual
    # current_sweep_radius_pixels = (angulo_barrido_rad / (2 * PI)) * display_radius_pixels
    # Para un efecto más realista de "ping", el radio debería expandirse rápidamente y luego reiniciarse
    # Vamos a simular un pulso que viaja hacia afuera
    # El tiempo que tarda el sonido en llegar al borde es R / V_sonido
    time_to_edge_seconds = s_max_current_range / SPEED_OF_SOUND_MPS
    # El tiempo actual en el ciclo de barrido (0 a sweep_duration_seconds)
    current_sweep_time = (angulo_barrido_rad / (2 * PI)) * sweep_duration_seconds
    
    # Ratio de tiempo actual vs tiempo para llegar al borde
    time_ratio = min(1.0, current_sweep_time / time_to_edge_seconds)
    
    # El radio del barrido visual se expande hasta el radio máximo
    # current_sweep_radius_pixels = time_ratio * display_radius_pixels
    
    # --- Dibujar la Rosa de los Vientos (Compás) ---
    effective_heading = current_ship_heading
    # Dibujar la rosa de los vientos con el rumbo efectivo
    # (El código para dibujar la rosa de los vientos se inserta aquí)

    # --- Dibujar Anillos de Rango ---
    # (El código para dibujar los anillos de rango se inserta aquí)

    # --- Dibujar Línea de Barrido ---
    # La línea de barrido ahora se dibuja desde el centro hasta el borde del círculo
    fin_x = circle_center_x + display_radius_pixels * math.sin(angulo_barrido_rad)
    fin_y = circle_center_y - display_radius_pixels * math.cos(angulo_barrido_rad)
    pygame.draw.line(pantalla, current_colors["SWEEP"], (circle_center_x, circle_center_y), (fin_x, fin_y), 2)

    # --- Dibujar Estela del Barco (Ship Track) ---
    # (El código para dibujar la estela del barco se inserta aquí)

    # --- Dibujar Marcadores de Objetivo ---
    # (El código para dibujar los marcadores de objetivo se inserta aquí)
    
    # --- Dibujar el Eco del Cardumen ---
    # Posición y tamaño fijos para la demostración
    distancia_cardumen_pixels = display_radius_pixels * 0.75  # A 3/4 del radio
    tamano_cardumen_pixels = 20
    angulo_cardumen_rad = math.radians(135) # Un ángulo fijo, p.ej. 135 grados

    # Obtener la potencia de TX actual desde el menú
    potencia_tx_actual = menu.options.get('potencia_tx', 8)

    # El "eco" solo es visible cuando el barrido pasa sobre él.
    # Comparamos el ángulo del barrido con el ángulo fijo del cardumen.
    # Usamos un umbral para que el eco no sea solo un punto instantáneo.
    angulo_diff = (angulo_barrido_rad - angulo_cardumen_rad + PI) % (2 * PI) - PI
    if abs(angulo_diff) < math.radians(10): # Un umbral de 10 grados para visibilidad
        dibujar_eco_cardumen(
            pantalla,
            circle_center_x,
            circle_center_y,
            angulo_cardumen_rad,  # Pasamos el ángulo fijo del cardumen
            distancia_cardumen_pixels,
            tamano_cardumen_pixels,
            potencia_tx_actual,
            display_radius_pixels
        )


    # --- Dibujar Paneles de Datos ---
    # (El código para dibujar los paneles de datos se inserta aquí)

    # --- Dibujar Menú ---
    menu.draw(pantalla)

    # --- Actualizar Pantalla ---
    pygame.display.flip()

    # Limita los fotogramas por segundo a 60
    reloj.tick(60)

# Finaliza Pygame
pygame.quit()
