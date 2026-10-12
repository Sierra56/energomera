import serial
import time

PORT = '/dev/ttyUSB0'

def test_handshake():
    print(f"Открытие порта {PORT}...")
    try:
        ser = serial.Serial(
            port=PORT,
            baudrate=9600,
            bytesize=serial.SEVENBITS,
            parity=serial.PARITY_EVEN,
            stopbits=serial.STOPBITS_ONE,
            timeout=1
        )
        
        print("🔌 Тест 1: Переключение DTR/RTS (имитация открытия терминала)...")
        ser.dtr = True
        ser.rts = True
        time.sleep(1)
        ser.dtr = False
        ser.rts = False
        time.sleep(1)
        
        print("👂 Слушаем порт 10 секунд...")
        print("⚠️ ВАЖНО: В это время нажмите и удерживайте кнопку 'ПРОСМОТР' или 'ГРУППА' на пульте CE901!")
        
        start_time = time.time()
        while time.time() - start_time < 10:
            if ser.in_waiting > 0:
                data = ser.read(ser.in_waiting)
                print(f"\n[{time.time() - start_time:.1f}с] ПОЛУЧЕНЫ ДАННЫЕ! ({len(data)} байт)")
                print("HEX:", data.hex(' '))
                print("ASCII:", data.decode('ascii', errors='replace'))
                break
            time.sleep(0.1)
        else:
            print("Данных не получено.")
            
        # Попробуем отправить запрос после рукопожатия
        print("\n📡 Отправка '/?!\\r\\n' после рукопожатия...")
        ser.write(b"/?!\r\n")
        time.sleep(2)
        
        if ser.in_waiting > 0:
            data = ser.read(ser.in_waiting)
            print("Ответ получен:", data.hex(' '))
        else:
            print("Ответ не получен.")
            
        ser.close()
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")

if __name__ == "__main__":
    test_handshake()
