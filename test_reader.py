import serial
import time
import re

# Параметры порта согласно документации CE901 (п. 4.1.7)
PORT = '/dev/ttyUSB0'
BAUDRATE = 9600
BYTESIZE = serial.SEVENBITS
PARITY = serial.PARITY_EVEN
STOPBITS = serial.STOPBITS_ONE

def read_meter():
    try:
        print(f"Открытие порта {PORT} (9600, 7E1)...")
        ser = serial.Serial(
            port=PORT,
            baudrate=BAUDRATE,
            bytesize=BYTESIZE,
            parity=PARITY,
            stopbits=STOPBITS,
            timeout=5
        )
        print("✅ Порт успешно открыт.")
        
        # Отправляем стандартный запрос идентификации IEC 61107
        request = b"/?!\r\n"
        print("Отправка запроса '/?!\\r\\n'...")
        ser.write(request)
        
        # Небольшая пауза для ответа устройства
        time.sleep(1.5)
        
        # Читаем ответ (буфер 2048 байт должен быть достаточным)
        response = ser.read(2048).decode('ascii', errors='ignore')
        
        print("\n--- Сырой ответ от устройства ---")
        print(repr(response)) # repr покажет все скрытые символы (\r, \n)
        print("---------------------------------\n")
        
        # Парсинг OBIS кодов (формат: X.X.X(значение*единица_измерения))
        pattern = r'(\d+\.\d+\.\d+)\(([^)]+)\)'
        matches = re.findall(pattern, response)
        
        if matches:
            print("✅ Распознанные показания (OBIS):")
            for obis, val in matches:
                # Извлекаем только числовое значение
                clean_val = re.search(r'([\d.]+)', val)
                if clean_val:
                    print(f"  OBIS {obis}: {clean_val.group(1)}")
        else:
            print("⚠️ OBIS коды не найдены в ответе. Возможно, устройство не успело считать данные или формат ответа отличается.")
            
        ser.close()
        print("Порт закрыт.")
        
    except Exception as e:
        print(f"❌ Ошибка: {e}")

if __name__ == "__main__":
    read_meter()
