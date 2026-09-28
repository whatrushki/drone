FROM python:3.11-slim

# Установка системных зависимостей для сборки и геопространственных вычислений
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    libgeos-dev \
    curl \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# Установка Python-зависимостей с кэшированием слоев
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Копирование исходного кода приложения и данных
COPY backend/ ./backend/
COPY frontend/ ./frontend/
COPY data/ ./data/
COPY run.py .

# Порт сервиса
EXPOSE 8000

# Healthcheck
HEALTHCHECK --interval=30s --timeout=5s --start-period=10s --retries=3 \
  CMD curl -f http://localhost:8000/api/health || exit 1

# Запуск приложения
CMD ["python", "run.py"]
