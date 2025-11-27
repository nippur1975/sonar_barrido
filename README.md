# Simulador de Sonar / Echosounder

Este es un simulador de sonar de barrido y ecosonda (fishfinder) desarrollado en Python utilizando Pygame.

## Requisitos

Para ejecutar este programa, necesitas tener instalado Python 3 y las siguientes bibliotecas:

*   `pygame`
*   `numpy`
*   `pyserial`
*   `geopy`

Puedes instalarlas ejecutando:

```bash
pip install pygame numpy pyserial geopy
```

## Archivos Necesarios

Asegúrate de que los siguientes archivos estén en la misma carpeta que `Sonar.py`:

*   **`Sonar.py`**: El script principal del programa.
*   **`DroidSansFallback.ttf`**: (Opcional pero recomendado) Archivo de fuente necesario para mostrar correctamente los caracteres en **Japonés** y **Coreano**.
    *   Si este archivo no está presente, el programa intentará usar una fuente del sistema, pero los caracteres asiáticos podrían no mostrarse correctamente.
*   **`eco.mp3`**: Archivo de sonido para el "ping" del sonar.

## Cómo Usar el Archivo de Fuente (.ttf)

El archivo `DroidSansFallback.ttf` se utiliza automáticamente por el programa para el soporte de idiomas CJK (Chino, Japonés, Coreano).

1.  **Ubicación:** Simplemente coloca el archivo `DroidSansFallback.ttf` en el mismo directorio que `Sonar.py`.
2.  **Selección de Idioma:**
    *   Ejecuta el programa.
    *   Presiona la tecla `M` para abrir el menú.
    *   Ve a la pestaña **SISTEMA**.
    *   Cambia la opción **IDIOMA** a `JAPANESE` o `KOREAN`.
    *   El programa cargará automáticamente la fuente `.ttf` para renderizar los textos en el idioma seleccionado.

## Ejecución

Para iniciar el simulador, ejecuta el siguiente comando en tu terminal:

```bash
python3 Sonar.py
```

## Controles Básicos

*   **M / ESC**: Abrir/Cerrar el menú.
*   **Flechas / H, J, K, L**: Navegación por el menú y ajustes (cuando el menú está activo).
*   **Clic Izquierdo**: Interactuar con las opciones del menú.
