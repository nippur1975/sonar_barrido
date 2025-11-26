# Importamos las bibliotecas pygame y math
import pygame
import math
import time
import numpy as np # Importar NumPy
import os # Needed for path manipulation
import random
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
try:
    pygame.mixer.init() # Initialize the mixer explicitly
except pygame.error:
    print("Warning: No audio device found. Audio disabled.")

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
            # Geopy can handle longitude wrapping, but good practice to keep it in standard range.
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
            {'label': 'ESCALA', 'key': 'sonda_escala', 'type': 'stepped_selector', 'values': [20, 40, 80, 120, 160, 240, 320]},
            {'label': 'DESPLAZAR ESC.', 'key': 'sonda_desplazar_esc', 'type': 'numeric_adjustable', 'range': (0, 1000)},
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
                    
                    elif focused_item['type'] in ['selector', 'stepped_selector']:
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
            elif item['type'] == 'stepped_selector':
                value_text = str(self.options[item['key']])
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

def draw_triangle(surface, color, center_x, center_y, size):
    """Draws a triangle shape pointing up."""
    half_size = size // 2
    points = [
        (center_x, center_y - half_size),  # Top point
        (center_x + half_size, center_y + half_size),  # Bottom-right
        (center_x - half_size, center_y + half_size)   # Bottom-left
    ]
    pygame.draw.polygon(surface, color, points, 2) # Border only

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
    #    both k could be positive (entry and exit) or complex (miss).
    
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
    #    both k could be positive (entry and exit) or complex (miss).
    
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

def draw_ship_track(surface, track_points_geo, ship_lat, ship_lon, ship_hdg_deg,
                    cc_x, cc_y, disp_radius_px, s_max_on_disp, current_disp_unit, track_color):
    # This is the new, optimized version of the function.
    if ship_lat is None or ship_lon is None:
        return

    # Safety check to prevent crash from invalid geo data
    if not -90 <= ship_lat <= 90 or not -180 <= ship_lon <= 180:
        print(f"DEBUG: Invalid coordinate in draw_ship_track. Lat: {ship_lat}, Lon: {ship_lon}. Skipping draw.")
        return

    # --- Step 1: Pre-calculation and Geo-to-Screen Conversion ---
    s_max_meters_on_display = s_max_on_disp
    if current_disp_unit == "BRAZAS":
        s_max_meters_on_display *= 1.8288

    current_ship_geo = Point(latitude=ship_lat, longitude=ship_lon)
    
    # Helper function to convert a single geo point to screen coords
    # This is where the expensive geodesic calls are contained.
    def geo_to_screen_coords(geo_pt, ref_ship_geo, ref_ship_hdg_deg, 
                             center_x, center_y, display_radius_pixels, 
                             max_dist_meters_for_display_edge):
        # This function calculates distance and bearing from ship to the point
        # and returns screen (x,y) or None if it's way off.
        dist_m = geodesic(ref_ship_geo, geo_pt).meters
        
        # A simple optimization: if a point is more than twice the sonar range,
        # it's definitely not going to be part of a visible segment.
        # This reduces the number of bearing calculations.
        if dist_m > max_dist_meters_for_display_edge * 2:
            return None, dist_m # Return distance for visibility checks

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
        return (int(round(scr_x)), int(round(scr_y))), dist_m

    # Process all points at once
    full_track_geo_points = track_points_geo + [{'lat': ship_lat, 'lon': ship_lon}]
    
    processed_points = []
    for geo_data in full_track_geo_points:
        geo_point = Point(latitude=geo_data['lat'], longitude=geo_data['lon'])
        screen_pos, dist_m = geo_to_screen_coords(geo_point, current_ship_geo, ship_hdg_deg,
                                                  cc_x, cc_y, disp_radius_px, s_max_meters_on_display)
        
        is_visible = dist_m <= s_max_meters_on_display
        processed_points.append({'screen_pos': screen_pos, 'is_visible': is_visible})

    # --- Step 2: Draw segments using only screen coordinates and 2D math ---
    if len(processed_points) < 2:
        return

    for i in range(len(processed_points) - 1):
        p1_info = processed_points[i]
        p2_info = processed_points[i+1]

        p1_screen = p1_info['screen_pos']
        p2_screen = p2_info['screen_pos']
        
        # Case 1: Both points are visible on screen. Draw the line.
        if p1_info['is_visible'] and p2_info['is_visible']:
            if p1_screen and p2_screen: # Should always be true if visible
                pygame.draw.line(surface, track_color, p1_screen, p2_screen, 1)
        
        # Case 2: One point visible, one not. Clip the line.
        # This requires both screen positions, even for the non-visible one, to get direction.
        elif p1_screen and p2_screen:
            # Determine which point is inside and which is outside
            inside_point = None
            outside_point = None
            if p1_info['is_visible'] and not p2_info['is_visible']:
                inside_point = p1_screen
                outside_point = p2_screen
            elif not p1_info['is_visible'] and p2_info['is_visible']:
                inside_point = p2_screen
                outside_point = p1_screen
            
            if inside_point and outside_point:
                # Use the 2D screen-based line clipping function
                intersection = get_screen_line_circle_intersection(
                    inside_point, outside_point,
                    cc_x, cc_y, disp_radius_px - 0.5 # a bit of inset
                )
                if intersection:
                    pygame.draw.line(surface, track_color, inside_point, intersection, 1)
        # Case 3: Both points are not visible.
        # For performance, we do not calculate if a segment with both ends off-screen
        # happens to pass through the circle. The visual impact of missing these
        # rare segments is negligible compared to the performance gain.

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

# --- Echosounder Simulation ---
VELOCIDAD_SONIDO = 1500.0  # m/s in salt water

# Furuno Palette
COLORES_ECO = [
    (0, 0, 30),         # 0: Deep blue sea background
    (0, 100, 255),      # 1: Blue (Noise)
    (0, 200, 200),      # 2: Cyan
    (0, 255, 0),        # 3: Green
    (255, 255, 0),      # 4: Yellow
    (255, 165, 0),      # 5: Orange
    (255, 100, 0),      # 6: Strong Orange
    (255, 0, 0),        # 7: Red (Bottom)
    (150, 0, 0),        # 8: Dark Red
    (100, 50, 0),       # 9: Brown
    (255, 255, 255)     # 10: WHITE LINE
]

class Echosounder:
    def __init__(self, width, height, colors):
        self.surface = pygame.Surface((width, height))
        self.distancia_barco = 0.0
        self.tiempo_acumulado = 0.0
        self.last_profundidad = 0.0
        self.font = pygame.font.SysFont("monospace", 16, bold=True)
        self.fuente_info = pygame.font.SysFont("monospace", 20, bold=True)
        self.resize(width, height, colors)

    def resize(self, width, height, colors):
        self.width = width
        self.height = height
        old_surface = self.surface if hasattr(self, 'surface') else None
        self.surface = pygame.Surface((width, height))
        self.surface.fill(colors["DATA_PANEL_BG"])
        if old_surface:
            self.surface.blit(old_surface, (0, 0))

    def _obtener_profundidad(self, config):
        base = 150.0  # Fixed base depth for simulation
        amp = 40.0
        
        onda_larga = np.sin(self.distancia_barco / 300.0) * amp
        onda_media = np.sin(self.distancia_barco / 120.0) * (amp * 0.3)
        ruido = np.sin(self.distancia_barco / 10.0) * 2.0
        
        prof_final = base + onda_larga + onda_media + ruido
        self.last_profundidad = max(10.0, prof_final)
        return self.last_profundidad

    def _dibujar_scanline(self, x, config):
        profundidad = self._obtener_profundidad(config)
        rango = config.get('sonda_escala', 300.0)
        ganancia = config.get('sonda_ganancia', 40) / 10.0
        filtro_parasit = config.get('sonda_filtro_parasit', 20) / 10.0
        shift = config.get('sonda_desplazar_esc', 0)

        if rango == 0: return
        factor_m_a_px = self.height / rango
        y_fondo = int((profundidad - shift) * factor_m_a_px)

        if y_fondo > 0:
            # Clutter simulation
            clutter_density = int(filtro_parasit / 10.0)
            if clutter_density > 0:
                clutter_points = np.random.randint(0, self.height, size=clutter_density)
                for y_clutter in clutter_points:
                    self.surface.set_at((x, y_clutter), COLORES_ECO[2]) # Pale blue
            
            # Interference simulation
            if config.get('sonda_rechz_interf', 'OFF') == 'ON':
                if random.random() < 0.1: # 10% chance of interference line
                    for y_interf in range(self.height):
                        if random.random() < 0.1:
                            self.surface.set_at((x, y_interf), COLORES_ECO[random.randint(1, 4)])

        grosor = int(15 + ganancia)
        anular_color_threshold = config.get('sonda_anular_color', 0) / 10.0
        
        curva_map = {'LINEAL': 1.0, '1': 1.0, '2': 0.8, '3': 0.6}
        gamma = curva_map.get(config.get('sonda_curva_color', 'LINEAL'), 1.0)

        for i in range(grosor):
            y_draw = y_fondo + i
            if 0 <= y_draw < self.height:
                raw_intensity = max(0, 1.0 - (i / grosor))

                if raw_intensity < anular_color_threshold:
                    continue

                displayed_intensity = raw_intensity ** gamma
                
                # Map intensity to color index (1-9)
                color_index = 1 + int(displayed_intensity * 8)
                color_index = max(1, min(9, color_index))
                
                self.surface.set_at((x, y_draw), COLORES_ECO[color_index])

    def update(self, dt_s, config, colors):
        rango = config.get('sonda_escala', 300.0)
        
        avance_map = {'2/1': 8.0, '1/1': 4.0, '1/2': 2.0, '1/4': 1.0, '1/8': 0.5}
        avance_key = config.get('sonda_avance_imagen', '1/1')
        avance = avance_map.get(avance_key, 4.0)

        if VELOCIDAD_SONIDO == 0: return
        tiempo_viaje_sonido = (rango * 2.0) / VELOCIDAD_SONIDO
        if avance == 0: return
        intervalo_ping = tiempo_viaje_sonido / avance
        
        self.tiempo_acumulado += dt_s
        
        if intervalo_ping > 0 and self.tiempo_acumulado >= intervalo_ping:
            lineas_a_dibujar = int(self.tiempo_acumulado / intervalo_ping)
            self.tiempo_acumulado -= (lineas_a_dibujar * intervalo_ping)
            lineas_a_dibujar = min(lineas_a_dibujar, 10)
            
            for _ in range(lineas_a_dibujar):
                self.surface.blit(self.surface, (-1, 0))
                pygame.draw.rect(self.surface, colors["DATA_PANEL_BG"], (self.width-1, 0, 1, self.height))
                self.distancia_barco += 0.5
                self._dibujar_scanline(self.width - 1, config)

    def draw(self, screen, dest_rect, config):
        screen.blit(pygame.transform.scale(self.surface, dest_rect.size), dest_rect)
        
        rango = config.get('sonda_escala', 300.0)
        if rango == 0: return
        factor = dest_rect.height / rango
        paso_grid = 50 if rango < 500 else 100
        for m in range(0, int(rango), paso_grid):
            y = dest_rect.top + m * factor
            pygame.draw.line(screen, (255, 255, 255, 50), (dest_rect.left, y), (dest_rect.right, y), 1)
            lbl = self.font.render(str(m), True, (150, 150, 150))
            screen.blit(lbl, (dest_rect.right - 40, y))
            
        draft = config.get('sonda_ajuste_calado', 0)
        txt_prof = self.fuente_info.render(f"{(self.last_profundidad + draft):.1f}m", True, (255, 255, 255))
        pygame.draw.rect(screen, (0,0,0), (dest_rect.left + 10, dest_rect.bottom - 40, 100, 30))
        screen.blit(txt_prof, (dest_rect.left + 20, dest_rect.bottom - 35))


# --- Clase Cardumen ---
class Cardumen:
    def __init__(self, lat_inicial, lon_inicial, profundidad_centro_inicial_m,
                 velocidad_nudos, curso_grados,
                 radio_horizontal_m, profundidad_superior_m, profundidad_inferior_m,
                 reflectividad_base=0.95):
        self.lat = lat_inicial
        self.lon = lon_inicial
        self.profundidad_centro = profundidad_centro_inicial_m
        self.velocidad_mps = velocidad_nudos * 0.514444
        self.curso_rad = math.radians(curso_grados)
        self.radio_horizontal = radio_horizontal_m
        self.profundidad_superior = profundidad_superior_m
        self.profundidad_inferior = profundidad_inferior_m
        self.altura_total = self.profundidad_inferior - self.profundidad_superior
        self.reflectividad_base = reflectividad_base
        self.x_sim = 0
        self.y_sim = 0
        self.avg_intensity = 0.0 # For Echo Average

    def actualizar_posicion(self, delta_tiempo_s, datos_nmea_disponibles=False):
        distancia_movimiento = self.velocidad_mps * delta_tiempo_s

        if datos_nmea_disponibles:
            start_point = Point(latitude=self.lat, longitude=self.lon)
            destination = geodesic(meters=distancia_movimiento).destination(point=start_point, bearing=math.degrees(self.curso_rad))
            self.lat = destination.latitude
            self.lon = destination.longitude
        else:
            dx_cardumen = distancia_movimiento * math.sin(self.curso_rad)
            dy_cardumen = distancia_movimiento * math.cos(self.curso_rad)

            self.x_sim += dx_cardumen
            self.y_sim += dy_cardumen


    def get_posicion_relativa_barco(self, barco_lat, barco_lon, barco_rumbo_deg, datos_nmea_disponibles=True):
        """
        Calcula la distancia horizontal, rumbo verdadero al cardumen desde el barco,
        y rumbo relativo al cardumen desde la proa del barco.
        """
        if datos_nmea_disponibles:
            if barco_lat is None or barco_lon is None: # Si NMEA está "disponible" pero no hay fix
                # Comportamiento como si NMEA no estuviera disponible para este cálculo
                return self._get_posicion_relativa_sin_nmea()

            punto_barco = Point(latitude=barco_lat, longitude=barco_lon)
            punto_cardumen = Point(latitude=self.lat, longitude=self.lon)

            dist_horizontal_m = geodesic(punto_barco, punto_cardumen).meters

            delta_lon_rad = math.radians(self.lon - barco_lon)
            lat_barco_rad = math.radians(barco_lat)
            lat_cardumen_rad = math.radians(self.lat)

            y_brg = math.sin(delta_lon_rad) * math.cos(lat_cardumen_rad)
            x_brg = math.cos(lat_barco_rad) * math.sin(lat_cardumen_rad) - \
                    math.sin(lat_barco_rad) * math.cos(lat_cardumen_rad) * math.cos(delta_lon_rad)
            
            rumbo_verdadero_a_cardumen_rad = math.atan2(y_brg, x_brg)
            rumbo_verdadero_a_cardumen_deg = (math.degrees(rumbo_verdadero_a_cardumen_rad) + 360) % 360
            
            rumbo_relativo_a_cardumen_deg = (rumbo_verdadero_a_cardumen_deg - barco_rumbo_deg + 360) % 360
            
            return {
                "dist_horizontal_m": dist_horizontal_m,
                "rumbo_verdadero_deg": rumbo_verdadero_a_cardumen_deg,
                "rumbo_relativo_deg": rumbo_relativo_a_cardumen_deg,
                "profundidad_centro_m": self.profundidad_centro,
                "profundidad_superior_m": self.profundidad_superior,
                "profundidad_inferior_m": self.profundidad_inferior,
                "radio_horizontal_m": self.radio_horizontal
            }
        else: # NMEA no disponible, usar simulación XY
            return self._get_posicion_relativa_sin_nmea()

    def _get_posicion_relativa_sin_nmea(self):
        # Barco en (0,0), rumbo 0° (Norte) en el plano simulado. Proa es +Y en pantalla.
        # Cardumen está en (self.x_sim, self.y_sim) en este plano (Norte es +Y, Este es +X).
        dist_horizontal_m = math.sqrt(self.x_sim**2 + self.y_sim**2)
        
        # Rumbo verdadero al cardumen: ángulo de (0,0) a (x_sim, y_sim)
        # atan2(dx, dy) donde dx es Este (+X), dy es Norte (+Y).
        rumbo_verdadero_a_cardumen_rad = math.atan2(self.x_sim, self.y_sim)
        rumbo_verdadero_a_cardumen_deg = (math.degrees(rumbo_verdadero_a_cardumen_rad) + 360) % 360
        
        # Barco simulado con rumbo 0° (Norte).
        # Rumbo relativo es el mismo que el rumbo verdadero en este caso.
        rumbo_relativo_a_cardumen_deg = rumbo_verdadero_a_cardumen_deg
            
        return {
            "dist_horizontal_m": dist_horizontal_m,
            "rumbo_verdadero_deg": rumbo_verdadero_a_cardumen_deg,
            "rumbo_relativo_deg": rumbo_relativo_a_cardumen_deg, # Relativo a la proa (que es Norte)
            "profundidad_centro_m": self.profundidad_centro,
            "profundidad_superior_m": self.profundidad_superior,
            "profundidad_inferior_m": self.profundidad_inferior,
            "radio_horizontal_m": self.radio_horizontal
        }

# --- Fin Clase Cardumen ---

# --- Lógica de Intersección Sonar-Cardumen ---
def calcular_interseccion_sonar_cardumen(pos_rel_cardumen, tilt_deg, apertura_haz_vertical_deg, max_rango_sonar_m, menu_options=None, cardumen_obj=None):
    """
    Calcula cómo el haz del sonar intersecta con el cardumen.
    Retorna un diccionario con parámetros de la intersección para el renderizado.
    
    pos_rel_cardumen: Diccionario de get_posicion_relativa_barco.
    tilt_deg: Ángulo de inclinación del haz del sonar (grados). 0 es horizontal. Positivo es hacia abajo.
    apertura_haz_vertical_deg: Apertura total vertical del haz del sonar (ej. 15 grados).
    max_rango_sonar_m: Alcance máximo actual del sonar en metros.
    menu_options: Diccionario con las opciones del menú para aplicar efectos (TVG, Gain, etc.).
    cardumen_obj: Objeto Cardumen para mantener estado (promedio).
    """
    dist_h = pos_rel_cardumen["dist_horizontal_m"]
    if dist_h == 0: # Evitar división por cero si el cardumen está directamente debajo
        dist_h = 1e-6 

    cardumen_prof_sup = pos_rel_cardumen["profundidad_superior_m"]
    cardumen_prof_inf = pos_rel_cardumen["profundidad_inferior_m"]
    cardumen_radio_h = pos_rel_cardumen["radio_horizontal_m"]

    # Convertir ángulos a radianes
    tilt_rad = math.radians(tilt_deg)
    media_apertura_haz_rad = math.radians(apertura_haz_vertical_deg / 2.0)

    # Límites angulares superior e inferior del haz del sonar en el plano vertical
    angulo_superior_haz_rad = tilt_rad - media_apertura_haz_rad
    angulo_inferior_haz_rad = tilt_rad + media_apertura_haz_rad

    # Profundidades que el haz del sonar cubre a la distancia horizontal del cardumen
    prof_sup_haz_en_dist_h = dist_h * math.tan(angulo_superior_haz_rad)
    prof_inf_haz_en_dist_h = dist_h * math.tan(angulo_inferior_haz_rad)
    
    # Asegurarse de que sup < inf si el haz es muy ancho o el tilt es extremo
    if prof_sup_haz_en_dist_h > prof_inf_haz_en_dist_h:
        prof_sup_haz_en_dist_h, prof_inf_haz_en_dist_h = prof_inf_haz_en_dist_h, prof_sup_haz_en_dist_h

    # Determinar el solapamiento vertical entre el haz y el cardumen
    solapamiento_vertical_sup = max(cardumen_prof_sup, prof_sup_haz_en_dist_h)
    solapamiento_vertical_inf = min(cardumen_prof_inf, prof_inf_haz_en_dist_h)
    
    altura_solapamiento_m = solapamiento_vertical_inf - solapamiento_vertical_sup

    if altura_solapamiento_m <= 0:
        return {"intensidad_factor": 0} # No hay solapamiento vertical

    # Calcular la "fuerza" del eco basado en el solapamiento vertical
    # Factor simple: proporción de la altura del cardumen que está iluminada
    # Se podría mejorar considerando la distribución de energía dentro del haz (gaussiana)
    factor_solapamiento_vertical = min(altura_solapamiento_m / (cardumen_prof_inf - cardumen_prof_sup), 1.0)
    
    # Calcular distancia inclinada (slant range) al centro del área de solapamiento
    profundidad_centro_solapamiento = (solapamiento_vertical_sup + solapamiento_vertical_inf) / 2.0
    dist_slant_centro_solapamiento = math.sqrt(dist_h**2 + profundidad_centro_solapamiento**2)

    if dist_slant_centro_solapamiento > max_rango_sonar_m :
        return {"intensidad_factor": 0} # El centro del solapamiento está fuera de rango

    # Atenuación por distancia (similar al ejemplo, pero usando slant range)
    # Usaremos una atenuación más simple por ahora, luego se puede ajustar
    # factor_atenuacion_distancia = np.exp(-dist_slant_centro_solapamiento / max_rango_sonar_m) # Exponencial
    # Una más simple: (1 - dist/max_dist)^2
    factor_atenuacion_distancia = (1.0 - (dist_slant_centro_solapamiento / max_rango_sonar_m))**2
    if dist_slant_centro_solapamiento > max_rango_sonar_m : factor_atenuacion_distancia = 0
    if dist_slant_centro_solapamiento == 0 : factor_atenuacion_distancia = 1 # Evitar div por cero si está justo en el origen


    # Calcular la anchura angular que el cardumen (su radio horizontal) subtiende en la pantalla
    # a la distancia horizontal dist_h. Esto es para la visualización en el PPI.
    if dist_h < cardumen_radio_h: # Si el barco está "dentro" del radio horizontal del cardumen
        media_anchura_angular_subtendida_rad = math.pi # Se vería en todas direcciones horizontales
    else:
        media_anchura_angular_subtendida_rad = math.atan(cardumen_radio_h / dist_h)
        # Esto es la mitad del ángulo, el total sería 2 * esto.

    # La "longitud" radial de la mancha en pantalla.
    # Podría ser la proyección de la altura del solapamiento en el slant range,
    # o una fracción del radio horizontal del cardumen.
    # Por ahora, usemos una fracción del radio, o un valor fijo.
    # O mejor, la proyección de la "profundidad" del cardumen a lo largo del haz
    # Distancia inclinada al tope del solapamiento y al fondo del solapamiento
    dist_slant_sup_solapamiento = math.sqrt(dist_h**2 + solapamiento_vertical_sup**2)
    dist_slant_inf_solapamiento = math.sqrt(dist_h**2 + solapamiento_vertical_inf**2)
    
    longitud_radial_mancha_m = abs(dist_slant_inf_solapamiento - dist_slant_sup_solapamiento)
    
    # Si el cardumen está muy cerca, la longitud radial podría ser grande.
    # Limitarla por el propio radio del cardumen también.
    longitud_radial_mancha_m = min(longitud_radial_mancha_m, cardumen_radio_h * 2)


    # Intensidad final combinada
    # El factor de "200tn" se considera en cardumen.reflectividad_base
    intensidad_final = factor_solapamiento_vertical * factor_atenuacion_distancia

    # --- Procesamiento de Señal (Menu Options) ---
    if menu_options:
        # 1. Potencia TX (1-10)
        potencia_tx = menu_options.get('potencia_tx', 8)
        intensidad_final *= (potencia_tx / 10.0)

        # 2. TVG (Time Varied Gain)
        norm_dist = dist_slant_centro_solapamiento / max_rango_sonar_m if max_rango_sonar_m > 0 else 0
        norm_dist = min(norm_dist, 1.0)

        tvg_near = menu_options.get('tvg_proximo', 6)
        if tvg_near > 0:
            suppression_factor = (tvg_near / 12.0) * math.exp(-norm_dist * 8) 
            intensidad_final *= (1.0 - suppression_factor)

        tvg_far = menu_options.get('tvg_lejano', 7)
        if tvg_far > 0:
            gain_boost = (tvg_far / 5.0) * norm_dist * norm_dist
            intensidad_final *= (1.0 + gain_boost)

        # 3. CAG (Control Automático de Ganancia)
        cag = menu_options.get('cag', 2)
        intensidad_final *= (0.8 + cag / 5.0)
        
        cag_2 = menu_options.get('cag_2', 1)
        intensidad_final *= (0.9 + cag_2 / 10.0)

        # 4. Longitud de Impulso
        long_impulso = menu_options.get('long_impulso', 8)
        extra_length = (long_impulso / 10.0) * 40.0 
        longitud_radial_mancha_m += extra_length
        intensidad_final *= (1.0 + long_impulso / 20.0)

        # 5. Limitar Ruido (Noise Limiter)
        # Suprime señales débiles (ruido de fondo simulado)
        limitar_ruido = menu_options.get('limitar_ruido', 3)
        if limitar_ruido > 0:
            noise_threshold = limitar_ruido / 20.0 # 0.05 to 0.5
            if intensidad_final < noise_threshold:
                intensidad_final = 0 # Supresión total bajo el umbral
            else:
                intensidad_final -= (noise_threshold * 0.5) # Reducción suave sobre el umbral

        # 6. Rechazo Interf (Interference Reject)
        # Simulamos reducción aleatoria o filtrado
        rechazo_interf = menu_options.get('rechazo_interf', 1) # 0-3
        if rechazo_interf > 0:
            # Simulación simple: reducir picos aleatorios o suavizar.
            # Aquí solo aplicamos una pequeña reducción constante para simular el filtrado agresivo.
            intensidad_final *= (1.0 - (rechazo_interf * 0.05))

        # 7. Promedio Eco (Echo Average)
        promedio_eco = menu_options.get('promedio_eco', 1) # 0-3
        if cardumen_obj:
            # Blend with history
            alpha = 1.0
            if promedio_eco == 1: alpha = 0.5
            elif promedio_eco == 2: alpha = 0.25
            elif promedio_eco == 3: alpha = 0.125
            
            if promedio_eco > 0:
                intensidad_final = (intensidad_final * alpha) + (cardumen_obj.avg_intensity * (1.0 - alpha))
            
            cardumen_obj.avg_intensity = intensidad_final

        # 8. Angulo Haz Hor (Horizontal Beamwidth)
        # ANCHO vs ESTRECHO
        angulo_haz_hor = menu_options.get('angulo_haz_hor', 'ANCHO')
        if angulo_haz_hor == 'ESTRECHO':
            # Reducir el ancho aparente (simular mayor resolución)
            media_anchura_angular_subtendida_rad *= 0.7
        else:
            # Ancho normal o aumentado
            media_anchura_angular_subtendida_rad *= 1.2

        # 9. Curva Color & Respuesta Color (Gamma Correction / Dynamic Range)
        curva_color = menu_options.get('curva_color', 1) # 1-4
        respuesta_color = menu_options.get('respuesta_color', 1) # 1-4
        
        # Curva: Afecta la gamma. 1=Lineal, >1=Expansión de bajos o altos.
        gamma = 1.0
        if curva_color == 2: gamma = 0.8
        elif curva_color == 3: gamma = 1.2
        elif curva_color == 4: gamma = 1.5
        
        if intensidad_final > 0:
            intensidad_final = pow(intensidad_final, gamma)

        # Respuesta: Afecta la ganancia dinámica (pendiente)
        if respuesta_color > 1:
            intensidad_final *= (1.0 + (respuesta_color - 1) * 0.1)

        # 10. Anular Color (Color Erase) - Check last to ensure finalized intensity
        anular_color = menu_options.get('anular_color', 0)
        if anular_color > 0:
            threshold = anular_color / 20.0 
            if intensidad_final < threshold:
                intensidad_final = 0

    return {
        "intensidad_factor": max(0, min(intensidad_final, 1.0)), # Clamp entre 0 y 1
        "dist_slant_m": dist_slant_centro_solapamiento, # Distancia inclinada al centro del eco
        "rumbo_relativo_deg": pos_rel_cardumen["rumbo_relativo_deg"], # Rumbo para colocarlo en PPI
        "media_anchura_angular_rad": media_anchura_angular_subtendida_rad, # Para el ancho de la mancha en PPI
        "longitud_radial_m": longitud_radial_mancha_m # Para la "profundidad" de la mancha en PPI
    }

# --- Fin Lógica de Intersección ---

# --- Cálculo de Matriz de Intensidad del Eco (Conceptual) ---
# Esta función es más conceptual. En lugar de generar una matriz densa como Z,
# la idea es que `calcular_interseccion_sonar_cardumen` ya nos da los parámetros
# clave (intensidad, posición central, extensión angular y radial)
# que usaremos directamente en la función de dibujo para generar la "mancha".
# No generaremos una matriz Z explícita para evitar sobrecarga, a menos que
# la función de dibujo demuestre necesitarla para un efecto muy específico.

# Por lo tanto, este paso del plan se fusiona en espíritu con el paso de renderizado.
# La "matriz de intensidad" será implícita en los parámetros devueltos por
# `calcular_interseccion_sonar_cardumen` y cómo se usan para dibujar.
# --- Fin Cálculo de Matriz de Intensidad ---

# --- Renderizado del Eco del Cardumen ---
def dibujar_eco_cardumen(surface, info_interseccion,
                          sonar_centro_x, sonar_centro_y,
                          sonar_radio_pixels, max_rango_sonar_en_metros,
                          current_gain,
                          color_base_rgb=(139, 0, 0)):
    """
    Dibuja el eco del cardumen en la pantalla del sonar.
    info_interseccion: Diccionario devuelto por calcular_interseccion_sonar_cardumen.
    sonar_radio_pixels: Radio en píxeles del display del sonar.
    max_rango_sonar_en_metros: Rango máximo actual del sonar CONVERTIDO A METROS.
    color_base_rgb: Color base para la máxima intensidad.
    """
    if not info_interseccion or info_interseccion.get("intensidad_factor", 0) <= 1e-3:
        return

    intensidad = info_interseccion["intensidad_factor"]
    dist_slant_m = info_interseccion["dist_slant_m"]
    rumbo_rel_deg = info_interseccion["rumbo_relativo_deg"]
    media_anchura_angular_rad = info_interseccion["media_anchura_angular_rad"]
    longitud_radial_m = info_interseccion["longitud_radial_m"]

    if max_rango_sonar_en_metros <= 0: return

    pixel_por_metro = sonar_radio_pixels / max_rango_sonar_en_metros
    dist_slant_pixels = dist_slant_m * pixel_por_metro
    
    if dist_slant_pixels > sonar_radio_pixels + 10:
        return

    longitud_radial_pixels = longitud_radial_m * pixel_por_metro
    longitud_radial_pixels_base = max(8, min(longitud_radial_pixels * 1.5, sonar_radio_pixels * 0.4))

    ancho_arco_pixels_base = 2 * dist_slant_pixels * math.tan(media_anchura_angular_rad)
    ancho_arco_pixels_base = max(5, min(ancho_arco_pixels_base, sonar_radio_pixels * 0.4))

    aumento_px_por_ganancia = current_gain 

    ancho_arco_pixels = ancho_arco_pixels_base + aumento_px_por_ganancia
    longitud_radial_pixels = longitud_radial_pixels_base + aumento_px_por_ganancia

    ancho_arco_pixels = max(1, ancho_arco_pixels)
    longitud_radial_pixels = max(1, longitud_radial_pixels)

    cuerpo_eco_color_base_rgb = (139, 0, 0)

    current_alpha_cuerpo = 255
    
    factor_brillo = 0.6 + 0.4 * intensidad
    
    final_r = int(cuerpo_eco_color_base_rgb[0] * factor_brillo)
    final_g = int(cuerpo_eco_color_base_rgb[1] * factor_brillo)
    final_b = int(cuerpo_eco_color_base_rgb[2] * factor_brillo)
    final_color_cuerpo_con_alfa = (final_r, final_g, final_b, current_alpha_cuerpo)

    angulo_dibujo_rad = math.radians(rumbo_rel_deg - 90)

    ancho_arco_pixels = 2 * dist_slant_pixels * math.tan(media_anchura_angular_rad)
    ancho_arco_pixels = max(5, min(ancho_arco_pixels, sonar_radio_pixels * 0.4))
    
    srf_size = int(max(ancho_arco_pixels, longitud_radial_pixels) * 1.5)
    if srf_size < 1: srf_size = 1
    eco_srf = pygame.Surface((srf_size, srf_size), pygame.SRCALPHA)
    eco_srf.fill((0,0,0,0))

    rect_x = int((srf_size - ancho_arco_pixels) / 2.0)
    rect_y = int((srf_size - longitud_radial_pixels) / 2.0)
    rect_w = int(ancho_arco_pixels)
    rect_h = int(longitud_radial_pixels)

    if rect_w <= 0: rect_w = 1
    if rect_h <= 0: rect_h = 1
    
    ellipse_rect_local = pygame.Rect(rect_x, rect_y, rect_w, rect_h)
    pygame.draw.ellipse(eco_srf, final_color_cuerpo_con_alfa, ellipse_rect_local)

    borde_alpha = 255

    colores_borde = [
        (0, 255, 0, borde_alpha),
        (255, 255, 0, borde_alpha),
        (255, 0, 0, borde_alpha)
    ]
    
    grosor_contorno_borde = 2
    
    rect_borde_rojo = ellipse_rect_local.inflate(grosor_contorno_borde * 2, grosor_contorno_borde * 2)
    pygame.draw.ellipse(eco_srf, colores_borde[2], rect_borde_rojo, grosor_contorno_borde)

    rect_borde_amarillo = rect_borde_rojo.inflate(grosor_contorno_borde * 2, grosor_contorno_borde * 2)
    pygame.draw.ellipse(eco_srf, colores_borde[1], rect_borde_amarillo, grosor_contorno_borde)
    
    rect_borde_verde = rect_borde_amarillo.inflate(grosor_contorno_borde * 2, grosor_contorno_borde * 2)
    pygame.draw.ellipse(eco_srf, colores_borde[0], rect_borde_verde, grosor_contorno_borde)

    rot_degrees = -math.degrees(angulo_dibujo_rad) - 90

    eco_rot_srf = pygame.transform.rotate(eco_srf, rot_degrees)
    
    centro_mancha_x = sonar_centro_x + dist_slant_pixels * math.cos(angulo_dibujo_rad)
    centro_mancha_y = sonar_centro_y + dist_slant_pixels * math.sin(angulo_dibujo_rad)
    
    eco_rot_rect = eco_rot_srf.get_rect(center=(centro_mancha_x, centro_mancha_y))

    dist_centro_mancha_al_origen_sonar = math.sqrt((centro_mancha_x - sonar_centro_x)**2 + (centro_mancha_y - sonar_centro_y)**2)
    
    if dist_centro_mancha_al_origen_sonar < sonar_radio_pixels + max(ancho_arco_pixels, longitud_radial_pixels)/2 :
        temp_sonar_circle_rect = pygame.Rect(sonar_centro_x - sonar_radio_pixels, sonar_centro_y - sonar_radio_pixels, 2*sonar_radio_pixels, 2*sonar_radio_pixels)
        if temp_sonar_circle_rect.colliderect(eco_rot_rect):
             surface.blit(eco_rot_srf, eco_rot_rect)

# --- Fin Renderizado del Eco del Cardumen ---

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

# --- Performance Optimization Variables ---
sweep_surface = None
last_sweep_surface_size = (0, 0)
# ---

# --- INICIALIZACIÓN PRINCIPAL ---
menu = MenuSystem()
load_settings()
# Sincronizar estado principal con las opciones del menú cargadas
current_unit = menu.options.get('unidad', 'METROS').upper()

# Cargar sonido y establecer volumen inicial desde la configuración
try:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    sound_file_path = os.path.join(script_dir, "eco.mp3")
    sonar_ping_sound = pygame.mixer.Sound(sound_file_path)
    initial_volume = menu.options.get('volumen_audio', 10) / 10.0
    sonar_ping_sound.set_volume(initial_volume)
except pygame.error as e:
    print(f"Advertencia: No se pudo cargar 'eco.mp3': {e}")
    sonar_ping_sound = None


# --- Auto-connect on Startup ---
if puerto:
    print(f"INFO: Se ha encontrado una configuración de puerto guardada. Intentando conectar a {puerto}@{baudios}...")
    try:
        ser = serial.Serial(puerto, baudios, timeout=1)
        serial_port_available = True
        print(f"INFO: Conectado automáticamente a {puerto}.")
    except serial.SerialException as e:
        print(f"ADVERTENCIA: No se pudo conectar automáticamente al puerto guardado {puerto}: {e}")
        puerto = None # Reset to avoid repeated failed attempts by other logic
        ser = None
        serial_port_available = False
# ---

# --- Inicialización del Cardumen ---
# Parámetros iniciales del cardumen
# Lat/Lon iniciales (si NMEA está disponible, sino se usan x_sim, y_sim)
# Para un barco en lat=0, lon=0, y el cardumen en la proa a 600m,
# si la proa es Norte (0°), el cardumen estaría en lat_cardumen_inicial, lon=0.
# geopy puede calcular esto: Point(0,0).destination(meters=600, bearing=0)
# Por ahora, si no hay NMEA, la posición geográfica no se usa activamente para el movimiento.
# Usaremos una lat/lon placeholder y nos centraremos en x_sim, y_sim.
lat_cardumen_placeholder = 0.0
lon_cardumen_placeholder = 0.0

# Profundidad centro: 40m (sup) + 60m (altura) / 2 = 70m desde superficie.
profundidad_centro_cardumen_m = 70
velocidad_cardumen_nudos = 4
curso_cardumen_grados = 190
radio_hor_cardumen_m = 200 / 2 # Diámetro 200m
prof_sup_cardumen_m = 40
prof_inf_cardumen_m = 100

cardumen_simulado = Cardumen(
    lat_inicial=lat_cardumen_placeholder,
    lon_inicial=lon_cardumen_placeholder,
    profundidad_centro_inicial_m=profundidad_centro_cardumen_m,
    velocidad_nudos=velocidad_cardumen_nudos,
    curso_grados=curso_cardumen_grados,
    radio_horizontal_m=radio_hor_cardumen_m,
    profundidad_superior_m=prof_sup_cardumen_m,
    profundidad_inferior_m=prof_inf_cardumen_m
)

# Posición inicial simulada del cardumen: 600m en proa.
# En nuestro sistema simulado sin NMEA, proa es +Y (Norte).
cardumen_simulado.x_sim = 0  # Directamente en proa
cardumen_simulado.y_sim = 600 # A 600m hacia el "Norte" relativo del barco
# --- Fin Inicialización del Cardumen ---

# --- Inicialización del Sistema Sonda ---
echosounder_sim = Echosounder(100, 100, current_colors) # Initial size, will be resized
# ---

# --- Estado de Inicialización Geográfica del Cardumen ---
cardumen_posicion_geografica_inicializada = False
# --- Fin Estado de Inicialización Geográfica del Cardumen ---

# --- Variables para el Retardo del Sonido del Eco ---
sound_play_time_cardumen = 0  # Momento en ms (pygame.time.get_ticks()) en que se debe reproducir el sonido del cardumen. 0 si no hay sonido programado.
sound_triggered_for_cardumen_echo = False # Para evitar múltiples disparos por el mismo eco de cardumen en un barrido.
# --- Fin Variables para el Retardo del Sonido ---

while not hecho:
    # Sincronizar variables que el menú puede cambiar
    current_unit = menu.options.get('unidad', 'METROS').upper()
    active_color_scheme_idx = menu.options.get('color', 3)
    current_colors = color_schemes[active_color_scheme_idx]
    nuevo_puerto = menu.options.get('puerto_com', None)
    nuevos_baudios = menu.options.get('port_baudios', 9600)

    # Lógica para reconectar el puerto serie si cambia en el menú
    if nuevo_puerto != puerto or nuevos_baudios != baudios:
        if ser:
            ser.close()
        puerto = nuevo_puerto
        baudios = nuevos_baudios
        if puerto and puerto != "Ninguno":
            try:
                ser = serial.Serial(puerto, baudios, timeout=1)
                serial_port_available = True
                print(f"Conectado al puerto {puerto} a {baudios} baudios.")
            except serial.SerialException as e:
                print(f"No se pudo conectar al puerto {puerto}: {e}")
                ser = None
                serial_port_available = False
        else:
            ser = None
            serial_port_available = False
    
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

    # 2. Layout para Sonar y Sonda
    modo_presentac = menu.options.get('modo_presentac', 'COMBI-1')
    
    sonar_margin_left = 10
    sonar_margin_top = 10
    sonar_margin_bottom = 10
    sonar_margin_right_to_panel = 10

    available_width_total_left = data_panel_x - sonar_margin_left - sonar_margin_right_to_panel
    available_height_total = dimensiones[1] - sonar_margin_top - sonar_margin_bottom
    
    sounder_rect = None # Rect for Sounder (Sonda)
    
    # Normal / COMBI-1 Logic (Sonar takes full left space)
    sonar_diameter = min(available_width_total_left, available_height_total)
    sonar_diameter = max(sonar_diameter, 200) 

    circle_width = sonar_diameter
    circle_height = sonar_diameter
    
    # Align top-left as before (or center vertically?)
    circle_origin_x = sonar_margin_left
    circle_origin_y = sonar_margin_top
    
    if modo_presentac in ['COMBI-1', 'COMBI-2']:
        # Sounder Logic: Place in the bottom part of the Data Panel
        # unified_data_box_dims = [x, y, w, h]
        data_panel_x = unified_data_box_dims[0]
        data_panel_y = unified_data_box_dims[1]
        data_panel_w = unified_data_box_dims[2]
        data_panel_h = unified_data_box_dims[3]
        
        # Estimate height used by text data ~350px
        data_text_height = 360 
        
        # Use remaining height for sounder, with some padding
        sounder_h = data_panel_h - data_text_height - 10
        if sounder_h > 100: # Ensure minimum height
            sounder_x = data_panel_x + 5
            sounder_y = data_panel_y + data_panel_h - sounder_h - 5
            sounder_w = data_panel_w - 10
            sounder_rect = pygame.Rect(sounder_x, sounder_y, sounder_w, sounder_h)
            unified_data_box_dims[3] = data_text_height # Reduce the height of the data text box
        
    circle_center_x = circle_origin_x + circle_width // 2
    circle_center_y = circle_origin_y + circle_height // 2
    display_radius_pixels = circle_width // 2

    # Calculate effective heading for use in this frame's calculations
    effective_heading = (current_ship_heading + menu.options.get('ajuste_proa', 0)) % 360

    # --- Mouse Tracking Logic ---
    mouse_x, mouse_y = pygame.mouse.get_pos()
    dist_to_center = math.sqrt((mouse_x - circle_center_x)**2 + (mouse_y - circle_center_y)**2)

    if not menu.active and dist_to_center <= display_radius_pixels:
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

            # --- NEW: Calculate Geographic Position of Cursor ---
            if current_ship_lat_deg is not None and current_ship_lon_deg is not None:
                # S_cursor is slant range in current units. Convert to meters for geo calculation.
                s_cursor_meters = S_cursor
                if current_unit == "BRAZAS":
                    s_cursor_meters *= 1.8288
                elif current_unit == "PIES":
                    s_cursor_meters *= 0.3048

                # bearing_deg_normalized is bearing relative to SCREEN UP.
                # We need bearing relative to TRUE NORTH.
                true_bearing_deg = (bearing_deg_normalized + effective_heading) % 360

                start_point = Point(latitude=current_ship_lat_deg, longitude=current_ship_lon_deg)
                destination = geodesic(meters=s_cursor_meters).destination(point=start_point, bearing=true_bearing_deg)

                lat_deg = destination.latitude
                lon_deg = destination.longitude

                lat_hem = 'N' if lat_deg >= 0 else 'S'
                lon_hem = 'E' if lon_deg >= 0 else 'W'
                
                lat_deg_abs = abs(lat_deg)
                lat_d = int(lat_deg_abs)
                lat_m = (lat_deg_abs - lat_d) * 60
                
                lon_deg_abs = abs(lon_deg)
                lon_d = int(lon_deg_abs)
                lon_m = (lon_deg_abs - lon_d) * 60

                ui_state["cursor_lat_str"] = f"{lat_d:02d}° {lat_m:06.3f}'{lat_hem}"
                ui_state["cursor_lon_str"] = f"{lon_d:03d}° {lon_m:06.3f}'{lon_hem}"
            else:
                ui_state["cursor_lat_str"] = "N/A"
                ui_state["cursor_lon_str"] = "N/A"
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
    update_marker_screen_positions(triangle_markers, current_ship_lat_deg, current_ship_lon_deg,
                                   current_ship_heading, circle_center_x, circle_center_y,
                                   display_radius_pixels, s_max_for_update, current_unit)
    # --- End Update Marker Screen Positions ---

    # --- Inicialización/Actualización de Posición Geográfica del Cardumen con NMEA ---
    if serial_port_available and current_ship_lat_deg is not None and current_ship_lon_deg is not None:
        if not cardumen_posicion_geografica_inicializada:
            # NMEA acaba de activarse con una posición válida, y el cardumen aún no ha sido posicionado geográficamente.
            distancia_sim_m = math.sqrt(cardumen_simulado.x_sim**2 + cardumen_simulado.y_sim**2)
            # Angulo de (0,0) a (x_sim, y_sim) en el plano simulado (Norte = 0 rad, Este = PI/2 rad)
            # atan2(dx, dy) -> atan2(x_sim, y_sim)
            bearing_sim_rad = math.atan2(cardumen_simulado.x_sim, cardumen_simulado.y_sim)
            # bearing_sim_deg es el rumbo relativo del cardumen respecto a la proa del barco (0° = proa)
            # en el sistema de coordenadas simulado (donde proa es +Y).
            # Si x_sim=0, y_sim=600 (proa), bearing_sim_rad = atan2(0,600) = 0 rad = 0 deg.
            # Si x_sim=600, y_sim=0 (estribor), bearing_sim_rad = atan2(600,0) = PI/2 rad = 90 deg.
            bearing_sim_deg = math.degrees(bearing_sim_rad) 

            # El rumbo verdadero al cardumen es el rumbo actual del barco + el rumbo relativo simulado.
            true_bearing_to_cardumen_deg = (current_ship_heading + bearing_sim_deg + 360) % 360
            
            start_point = Point(latitude=current_ship_lat_deg, longitude=current_ship_lon_deg)
            try:
                destination = geodesic(meters=distancia_sim_m).destination(point=start_point, bearing=true_bearing_to_cardumen_deg)
                cardumen_simulado.lat = destination.latitude
                cardumen_simulado.lon = destination.longitude
                cardumen_posicion_geografica_inicializada = True
                print(f"INFO: Cardumen inicializado geográficamente en Lat: {cardumen_simulado.lat:.4f}, Lon: {cardumen_simulado.lon:.4f} (Dist: {distancia_sim_m:.1f}m, Bearing: {true_bearing_to_cardumen_deg:.1f}° from ship at {current_ship_lat_deg:.4f},{current_ship_lon_deg:.4f} heading {current_ship_heading:.1f}°)")
            except Exception as e_geo_init:
                print(f"ERROR: Fallo al calcular destino geográfico inicial del cardumen: {e_geo_init}")
                # El cardumen permanecerá en su lat/lon placeholder (0,0) hasta el próximo intento exitoso.
                # cardumen_posicion_geografica_inicializada permanecerá False.

    elif not serial_port_available or current_ship_lat_deg is None or current_ship_lon_deg is None:
        # NMEA se ha perdido o no está disponible. Reseteamos el flag.
        if cardumen_posicion_geografica_inicializada:
            print("INFO: Conexión NMEA perdida o inválida. Cardumen volverá a modo simulación XY si NMEA se reactiva.")
            # Aquí podríamos querer que x_sim, y_sim se actualicen para reflejar la última posición relativa
            # conocida geográficamente, para una transición más suave si NMEA vuelve.
            # Por ahora, simplemente reseteamos el flag.
            cardumen_posicion_geografica_inicializada = False
    # --- Fin Inicialización/Actualización de Posición Geográfica del Cardumen ---

    # --- Hover Logic for Target Markers ---
    ui_state['hovered_marker_index'] = None # Reset hover state each frame
    ui_state['hovered_marker_list'] = None
    if not menu.active:
        # Reset hover state for all markers first
        for marker in target_markers: marker['is_hovered'] = False
        for marker in triangle_markers: marker['is_hovered'] = False

        # Check target_markers
        for i, marker_data in enumerate(target_markers):
            if marker_data['current_screen_pos']:
                mx, my = marker_data['current_screen_pos']
                marker_icon_size = 18
                hover_rect = pygame.Rect(mx - marker_icon_size // 2, my - marker_icon_size // 2, marker_icon_size, marker_icon_size)
                if hover_rect.collidepoint(mouse_x, mouse_y):
                    ui_state['hovered_marker_index'] = i
                    ui_state['hovered_marker_list'] = 'target'
                    marker_data['is_hovered'] = True
                    break
        
        # If no hover found yet, check triangle_markers
        if ui_state['hovered_marker_index'] is None:
            for i, marker_data in enumerate(triangle_markers):
                if marker_data['current_screen_pos']:
                    mx, my = marker_data['current_screen_pos']
                    marker_icon_size = 18
                    hover_rect = pygame.Rect(mx - marker_icon_size // 2, my - marker_icon_size // 2, marker_icon_size, marker_icon_size)
                    if hover_rect.collidepoint(mouse_x, mouse_y):
                        ui_state['hovered_marker_index'] = i
                        ui_state['hovered_marker_list'] = 'triangle'
                        marker_data['is_hovered'] = True
                        break
    # --- End Hover Logic ---

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
        
        # Ajuste por Ciclo TX (Intervalo)
        # Ciclo TX 10 = Normal. Menor valor = Más lento (o pausas).
        # Simulamos ralentizando el barrido.
        ciclo_tx_val = menu.options.get('ciclo_tx', 10)
        # Factor: 10 -> 1.0, 1 -> 0.1
        factor_ciclo = max(0.1, ciclo_tx_val / 10.0)
        # Invertimos el efecto en el tiempo: Menor ciclo = Tiempo efectivo más largo (más lento)
        # O simplemente reducimos la velocidad de píxeles por segundo.
        
        # Pixels per second for the sweep to reach the edge in effective_time_for_sweep_s
        sweep_pixels_per_second = display_radius_pixels / effective_time_for_sweep_s
        
        # Aplicar factor de ciclo
        sweep_pixels_per_second *= factor_ciclo

        sweep_increment_ppf = sweep_pixels_per_second / FPS
    
    # --- Sonar Sweep Animation Logic ---
    if menu.options.get("transmision") == "ON":
        current_sweep_radius_pixels += sweep_increment_ppf
        if current_sweep_radius_pixels > display_radius_pixels:
            current_sweep_radius_pixels = 0 # Reset sweep
            sound_triggered_for_cardumen_echo = False # Permitir nuevo disparo de sonido para el próximo barrido del cardumen
            if sonar_ping_sound: # Restaurar el sonido original del ciclo de barrido
                sonar_ping_sound.play() 
    else:
        current_sweep_radius_pixels = 0
        if sonar_ping_sound and pygame.mixer.get_busy(): 
            sonar_ping_sound.stop() 
            
    # --- End Sonar Sweep Animation Logic ---
    # --- End Sonar Sweep Parameter Calculation ---

    # --- Actualización y Lógica del Cardumen ---
    delta_tiempo_s = reloj.get_time() / 1000.0  # Tiempo desde el último frame en segundos
    
    # Determinar si hay datos NMEA válidos para la actualización del cardumen
    # (current_ship_lat_deg no es None y current_ship_lon_deg no es None)
    # Y serial_port_available es True.
    nmea_para_cardumen = serial_port_available and current_ship_lat_deg is not None and current_ship_lon_deg is not None

    cardumen_simulado.actualizar_posicion(delta_tiempo_s, datos_nmea_disponibles=nmea_para_cardumen)

    # Obtener posición relativa del cardumen para la lógica de intersección y dibujo
    # Usar current_ship_heading (que es 0.0 si no hay NMEA)
    pos_rel_cardumen = cardumen_simulado.get_posicion_relativa_barco(
        current_ship_lat_deg, # Puede ser None
        current_ship_lon_deg, # Puede ser None
        current_ship_heading, # Es 0.0 si no hay NMEA
        datos_nmea_disponibles=nmea_para_cardumen
    )

    # Calcular intersección
    # Necesitamos el rango máximo del sonar EN METROS
    max_rango_actual_unidades = range_presets_map[current_unit][current_range_index]
    max_rango_actual_metros = max_rango_actual_unidades
    if current_unit == "BRAZAS":
        max_rango_actual_metros *= 1.8288
    
    # Conectar la opción del menú 'angulo_haz_ver' a la lógica
    angulo_haz_ver_str = menu.options.get('angulo_haz_ver', 'ANCHO')
    apertura_haz_vertical_deg = 15.0 if angulo_haz_ver_str == 'ANCHO' else 7.5

    info_interseccion_cardumen = calcular_interseccion_sonar_cardumen(
        pos_rel_cardumen,
        current_tilt_angle, # El tilt actual del sonar
        apertura_haz_vertical_deg,
        max_rango_actual_metros,
        menu.options, # Pasar opciones del menú
        cardumen_simulado # Pasar objeto cardumen
    )
    
    # --- Update Echosounder System ---
    if modo_presentac in ['COMBI-1', 'COMBI-2']:
        if sounder_rect and (sounder_rect.width != echosounder_sim.width or sounder_rect.height != echosounder_sim.height):
             echosounder_sim.resize(sounder_rect.width, sounder_rect.height, current_colors)
        echosounder_sim.update(delta_tiempo_s, menu.options, current_colors)
    # --- Fin Actualización y Lógica del Cardumen ---

    # --- Lógica de Programación y Reproducción del Sonido del Eco con Retardo ---
    if menu.options.get("transmision") == "ON" and sonar_ping_sound and \
       'info_interseccion_cardumen' in locals() and info_interseccion_cardumen and \
       info_interseccion_cardumen.get("intensidad_factor", 0) > 0.1: # Solo si hay un eco significativo

        dist_eco_m = info_interseccion_cardumen["dist_slant_m"]
        
        # Convertir dist_eco_m a píxeles para comparar con current_sweep_radius_pixels
        # Necesitamos max_rango_actual_metros y display_radius_pixels
        if max_rango_actual_metros > 0:
            pixel_por_metro_eco = display_radius_pixels / max_rango_actual_metros
            dist_eco_pixels = dist_eco_m * pixel_por_metro_eco

            # Comprobar si el barrido visual está "cerca" del eco y el sonido no ha sido disparado aún para este eco en este barrido
            # Usamos un pequeño umbral para la detección (ej., +/- sweep_increment_ppf)
            if not sound_triggered_for_cardumen_echo and \
               abs(current_sweep_radius_pixels - dist_eco_pixels) < (sweep_increment_ppf * 2 + 5) and \
               current_sweep_radius_pixels <= dist_eco_pixels + sweep_increment_ppf: # Asegura que el barrido no haya pasado mucho más allá

                retardo_ms = (dist_eco_m / SPEED_OF_SOUND_MPS) * 1000
                sound_play_time_cardumen = pygame.time.get_ticks() + retardo_ms
                sound_triggered_for_cardumen_echo = True
                # print(f"Eco CARDUMEN detectado a {dist_eco_m:.2f}m. Sonido programado en {retardo_ms:.2f}ms. Play at: {sound_play_time_cardumen}") # DEBUG

    # Comprobar si es momento de reproducir un sonido programado para el cardumen
    if sound_play_time_cardumen > 0 and pygame.time.get_ticks() >= sound_play_time_cardumen:
        if sonar_ping_sound: # Usamos el mismo sonido por ahora
            sonar_ping_sound.play()
            # print(f"SONIDO CARDUMEN REPRODUCIDO en {pygame.time.get_ticks()}") # DEBUG
        sound_play_time_cardumen = 0 # Resetear para que no se reproduzca de nuevo inmediatamente
    # --- Fin Lógica de Sonido del Eco ---

    # --- Lógica de Auto Tilt (Inclinación Automática) ---
    if menu.options.get('inclin_auto') == 'ON':
        # Oscilar el tilt automáticamente
        # Usamos el tiempo para una oscilación suave
        # Periodo de oscilación: ~10 segundos
        oscillation_speed = 0.002 # Ajustar velocidad
        tilt_base = 15 # Centro de oscilación
        
        # Determinar amplitud basada en 'angulo_inclin'
        # Valores: '±2-10°', '±4-14°', etc. Simplificamos parseo.
        angulo_inclin_opt = menu.options.get('angulo_inclin', '±2-10°')
        tilt_amplitude = 5 # Default
        if '2-10' in angulo_inclin_opt: tilt_amplitude = 4
        elif '4-14' in angulo_inclin_opt: tilt_amplitude = 5
        elif '6-20' in angulo_inclin_opt: tilt_amplitude = 7
        elif '10-26' in angulo_inclin_opt: tilt_amplitude = 8
        
        # Calcular nuevo tilt
        current_time_s = time.time()
        tilt_offset = math.sin(current_time_s * 0.5) * tilt_amplitude # 0.5 rad/s frequency
        
        # Actualizar current_tilt_angle (clamped)
        current_tilt_angle = int(tilt_base + tilt_offset)
        current_tilt_angle = max(MIN_TILT, min(MAX_TILT, current_tilt_angle))
        
        # Mostrar en pantalla temporalmente para feedback visual
        show_tilt_temporarily = True
        tilt_display_timer = 2 # Mantenerlo visible

    # --- Lógica de Alarma (Alarm Level) ---
    # Si la intensidad del eco supera el nivel de alarma, mostrar alerta.
    nivel_alarma = menu.options.get('nivel_alarma', 9) # 1-10
    alarm_threshold = nivel_alarma / 10.0
    
    if 'info_interseccion_cardumen' in locals() and info_interseccion_cardumen:
        if info_interseccion_cardumen.get("intensidad_factor", 0) >= alarm_threshold and alarm_threshold > 0:
            # Dibujar indicador de alarma visual
            alarm_text = font_very_large.render("ALARM!", True, current_colors["TARGET_HOVER"]) # Rojo
            alarm_rect = alarm_text.get_rect(center=(center_x, center_y - 50))
            pantalla.blit(alarm_text, alarm_rect)
            
            # Opcional: Reproducir sonido si no está sonando ya
            # if sonar_ping_sound: sonar_ping_sound.play() # Puede ser molesto si es continuo

    for evento in pygame.event.get():
        if evento.type == pygame.QUIT:
            hecho = True
        
        action = menu.handle_event(evento)
        if action == 'clear_markers':
            target_markers.clear()
            print("INFO: Todas las marcas de derrota han sido borradas.")

        if not menu.active:
            if evento.type == pygame.VIDEORESIZE:
                dimensiones[0], dimensiones[1] = evento.w, evento.h
                print(f"Ventana redimensionada a: {dimensiones}") # Debug
            
            if evento.type == pygame.KEYDOWN:
                if current_range_index >= len(range_presets_map[current_unit]):
                    current_range_index = len(range_presets_map[current_unit]) - 1
                s_max_for_event = range_presets_map[current_unit][current_range_index]

                handle_key_events(evento.key, 
                                  circle_center_x, circle_center_y, 
                                  display_radius_pixels, s_max_for_event)
            
            if evento.type == pygame.MOUSEBUTTONDOWN:
                # Aquí iría la lógica de clic del ratón para la pantalla principal (ej. añadir marcadores)
                pass


    # Read from serial port if available
    if serial_port_available and ser: # Check ser object directly
        try:
            if ser.in_waiting > 0:
                line = ser.readline().decode('ascii', errors='replace').strip()
                
                # Process only if a line was actually received
                if line and is_valid_nmea_checksum(line):
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
                elif line:
                    # Optional: Print discarded sentences for debugging
                    print(f"Discarding corrupt NMEA sentence: {line}")
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
                true_marker_bearing_deg = (effective_heading + screen_bearing_deg + 360) % 360
                
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
    brightness_factor = menu.options.get('iluminacion', 5) / 10.0
    bg_color_base = current_colors["BACKGROUND"]
    bg_color_adjusted = tuple(max(0, min(255, int(c * brightness_factor))) for c in bg_color_base)
    pantalla.fill(bg_color_adjusted)


    # --- Display Cursor Lat/Lon at Upper Right Center ---
    if ui_state["show_plus_cursor"]:
        lat_surf = font.render(ui_state["cursor_lat_str"], True, current_colors["PRIMARY_TEXT"])
        lon_surf = font.render(ui_state["cursor_lon_str"], True, current_colors["PRIMARY_TEXT"])
        
        # Position in the center of the right-hand half of the screen, at the top.
        center_x_pos = int(dimensiones[0] * 0.75) - 300

        lat_rect = lat_surf.get_rect(centerx=center_x_pos, top=10)
        lon_rect = lon_surf.get_rect(centerx=center_x_pos, top=lat_rect.bottom + 2)
        
        pantalla.blit(lat_surf, lat_rect)
        pantalla.blit(lon_surf, lon_rect)


    # Usar las variables calculadas dinámicamente para el círculo del sonar
    dimensiones_caja = [circle_origin_x, circle_origin_y, circle_width, circle_height]
    # Dibujamos el borde de un círculo para 'barrerlo'
    pygame.draw.ellipse(pantalla, current_colors["PRIMARY_TEXT"], dimensiones_caja, 2)

    # --- Draw Sounder (Sonda) ---
    if sounder_rect and modo_presentac in ['COMBI-1', 'COMBI-2']:
        echosounder_sim.draw(pantalla, sounder_rect, menu.options)
    # --- End Draw Sounder ---

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
        # Optimization: Re-create surface only on size change
        current_sweep_surface_size = (display_radius_pixels * 2, display_radius_pixels * 2)
        if sweep_surface is None or current_sweep_surface_size != last_sweep_surface_size:
            if current_sweep_surface_size[0] > 0 and current_sweep_surface_size[1] > 0:
                sweep_surface = pygame.Surface(current_sweep_surface_size, pygame.SRCALPHA)
                last_sweep_surface_size = current_sweep_surface_size
        
        if sweep_surface:
            # Draw a semi-transparent blue circle for the sweep
            sweep_surface.fill((0, 0, 0, 0)) # Clear the surface for transparency
            sweep_color_with_alpha = (*current_colors["SWEEP"], 150)
            pygame.draw.circle(sweep_surface, sweep_color_with_alpha,
                               (display_radius_pixels, display_radius_pixels), # Center of the temp surface
                               int(current_sweep_radius_pixels), 2) # Thickness 2
            pantalla.blit(sweep_surface, (center_x - display_radius_pixels, center_y - display_radius_pixels))
    # --- End Draw Sonar Sweep ---

    # Draw the center icon
    draw_center_icon(pantalla, center_x, center_y, 36, current_colors["CENTER_ICON"]) # Changed height to 36, thickness remains 5

    # --- Dibujar Eco del Cardumen (MOVIDO ANTES DE ELEMENTOS DE UI SUPERPUESTOS) ---
    if menu.options.get("transmision") == "ON":
        if 'info_interseccion_cardumen' in locals() and info_interseccion_cardumen:
            dibujar_eco_cardumen(
                pantalla,
                info_interseccion_cardumen,
                circle_center_x,
                circle_center_y,
                display_radius_pixels,
                max_rango_actual_metros,
                current_gain
            )
    # --- Fin Dibujar Eco del Cardumen ---

    # --- End Display Calculated Target Data ---

    # --- Draw Sonar Rose Unit Text (Bottom-Right of Sonar Rose, above target data if visible) ---
    # Re-render active_sonar_rose_unit_surface with current colors
    active_sonar_rose_unit_surface = font.render(sonar_rose_unit_text_map[current_unit], True, current_colors["PRIMARY_TEXT"])

    if active_sonar_rose_unit_surface:
        sonar_rose_unit_rect = active_sonar_rose_unit_surface.get_rect()
        
        # New positioning logic:
        # Align with the R/T/G text horizontally (right-aligned, 10px left of the data panel)
        sonar_rose_unit_rect.right = unified_data_box_dims[0] - 10

        # Align with the bottom of the sonar circle vertically
        sonar_rose_unit_rect.bottom = dimensiones_caja[1] + dimensiones_caja[3] - 5 # 5px padding from bottom

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
    if menu.options.get('mensaje_veloc') == 'ON':
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

    # --- OLD DATA PANEL DRAWING REMOVED ---

    # --- OLD BUTTON DRAWING REMOVED ---

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

    if menu.options.get('indi_subtexto') == 'ON':
        # --- Display Calculated Target Data (Bottom-Right inside Sonar Rose) ---
        large_symbol_font = font_large 
        label_font = font 
        
        if len(target_markers) >= 2:
            effective_line_height = large_symbol_font.get_linesize() 
            
            margin_bottom_sonar_circle = 10 
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
            total_text_block_height = (num_lines_target_data * effective_line_height) + \
                                        ((num_lines_target_data - 1) * 3 if num_lines_target_data > 0 else 0)
    
            start_y_for_target_block = (circle_origin_y + circle_height) - total_text_block_height - margin_bottom_sonar_circle
            
            current_y_offset_target_data = start_y_for_target_block
    
            # Calculate the maximum width of the rendered lines to ensure right-alignment
            max_line_width = 0
            rendered_lines = []
            for label_str, value_str, is_special in target_data_lines_info:
                font_for_label = large_symbol_font if is_special else label_font
                label_surf = font_for_label.render(label_str, True, current_colors["PRIMARY_TEXT"])
                value_surf = label_font.render(value_str, True, current_colors["PRIMARY_TEXT"])
                line_width = label_surf.get_width() + symbol_value_spacing + value_surf.get_width()
                if line_width > max_line_width:
                    max_line_width = line_width
                rendered_lines.append({'label_surf': label_surf, 'value_surf': value_surf, 'is_special': is_special})
            
            # --- AHORA LA PARTE MODIFICADA: POSICIONAR EL BLOQUE RESPECTO AL PANEL DE DATOS ---
            margin_right_to_data_panel = 10
            right_x_for_target_block = unified_data_box_dims[0] - margin_right_to_data_panel
    
            for i, line_info in enumerate(rendered_lines):
                label_surf = line_info['label_surf']
                value_surf = line_info['value_surf']
                
                current_line_total_width = label_surf.get_width() + symbol_value_spacing + value_surf.get_width()
                current_start_x_for_line = right_x_for_target_block - current_line_total_width
                
                y_pos_for_this_line = current_y_offset_target_data + (i * (effective_line_height + 3))
    
                label_rect = label_surf.get_rect(topleft=(current_start_x_for_line, y_pos_for_this_line))
                value_rect = value_surf.get_rect(left=label_rect.right + symbol_value_spacing, centery=label_rect.centery)
                
                pantalla.blit(label_surf, label_rect)
                pantalla.blit(value_surf, value_rect)
    
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

    # Draw triangle markers separately
    for marker in triangle_markers:
        draw_pos = None
        if marker['mode'] == 'geo':
            if marker['current_screen_pos']:
                mx_geo, my_geo = marker['current_screen_pos']
                dist_sq_to_center = (mx_geo - circle_center_x)**2 + (my_geo - circle_center_y)**2
                if dist_sq_to_center <= (display_radius_pixels + marker_icon_size/2)**2:
                    draw_pos = marker['current_screen_pos']
        elif marker['mode'] == 'screen':
            draw_pos = marker['current_screen_pos']

        if draw_pos:
            mx, my = draw_pos
            current_marker_color = current_colors["TARGET_HOVER"] if marker.get('is_hovered', False) else current_colors["TARGET_BASE"]
            draw_triangle(pantalla, current_marker_color, mx, my, marker_icon_size)

    # Draw lines between consecutive target markers (Rhombus/X only)
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
                hypothetical_m2_y = circle_center_y + far_radius * math.sin(angle_rad_for_draw)
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

    # --- OLD POPUP DRAWING LOGIC REMOVED ---

    # --- Display Coords for last Triangle Marker ---
    if triangle_markers:
        last_marker = triangle_markers[-1]
        lat_str = "N/A"
        lon_str = "N/A"
        if last_marker.get('geo_pos'):
            lat = last_marker['geo_pos']['lat']
            lon = last_marker['geo_pos']['lon']
            # Reusing the formatting logic from the cursor display
            lat_hem = 'N' if lat >= 0 else 'S'
            lon_hem = 'E' if lon >= 0 else 'W'
            lat_deg_abs = abs(lat)
            lat_d = int(lat_deg_abs)
            lat_m = (lat_deg_abs - lat_d) * 60
            lon_deg_abs = abs(lon)
            lon_d = int(lon_deg_abs)
            lon_m = (lon_deg_abs - lon_d) * 60
            lat_str = f"{lat_d:02d}° {lat_m:06.3f}'{lat_hem}"
            lon_str = f"{lon_d:03d}° {lon_m:06.3f}'{lon_hem}"
        
        lat_surf = font.render(lat_str, True, current_colors["PRIMARY_TEXT"])
        lon_surf = font.render(lon_str, True, current_colors["PRIMARY_TEXT"])
        
        lon_rect = lon_surf.get_rect(bottomleft=(10, dimensiones[1] - 10))
        lat_rect = lat_surf.get_rect(bottomleft=(10, lon_rect.top - 2))

        pantalla.blit(lat_surf, lat_rect)
        pantalla.blit(lon_surf, lon_rect)

    # Avancemos y actualicemos la pantalla con lo que hemos dibujado.
    menu.draw(pantalla)
    pygame.display.flip()
    pygame.image.save(pantalla, "sonar_screenshot.png")

    # Limitamos a 60 fotogramas por segundo
    reloj.tick(60)

if serial_port_available and ser is not None:
    ser.close()

# --- Save Settings on Exit ---
save_settings()
# ---
pygame.quit()





