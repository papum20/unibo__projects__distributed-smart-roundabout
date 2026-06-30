FROM python:3.12-slim

WORKDIR /app

COPY service-vehicle/ .
COPY common/ common/

RUN pip install --no-cache-dir -r requirements.txt

# Run the script
CMD ["python", "main.py"]