import serial
import time

PORT = '/dev/ttyUSB0'
BAUDRATE = 9600
BYTESIZE = serial.SEVENBITS
PARITY = serial.PARITY_EVEN
STOPBITS = serial.STOPBITS_ONE

def sniff_port():
    print(f"Открытие порта {PORT} (9600, 7E1)...")
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=BYTESIZE,
            parity=PARITY,
            stopbits=STOPBITS,
            timeout=1 # Читаем по 1 секунде
        )
        print("✅ Порт открыт. Начинаем прослушивание эфира (15 секунд)...")
        print("Пожалуйста, поднесите пульт к счетчику, если он еще не рядом.")
        
        start_time = time.time()
        duration = 15 # секунд
        
        while time.time() - start_time < duration:
            # Читаем все доступные байты
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"\n[{time.time() - start_time:.1f}с] Получено {len(data)} байт:")
                print("HEX:", data.hex(' '))
                print("ASCII:", data.decode('ascii', errors='replace'))
                print("-" * 40)
            time.sleep(0.1)
            
        print("\n⏹️ Прослушивание завершено.")
        
        # Теперь попробуем отправить запрос и подождать
        print("\n📡 Отправка запроса '/?!\\r\\n' и ожидание ответа (10 сек)...")
        ser.reset_input_buffer() # Очищаем буфер
        ser.write(b"/?!\r\n")
        
        start_time = time.time()
        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"\n[{time.time() - start_time:.1f}с] Ответ: {len(data)} байт:")
                print("HEX:", data.hex(' '))
                print("ASCII:", data.decode('ascii', errors='replace'))
                break
            time.sleep(0.1)
        else:
            print("️ Ответ не получен в течение 10 секунд.")
            
        ser.close()
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")

if __name__ == "__main__":
    sniff_port()
