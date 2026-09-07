FROM python:3.12-slim

WORKDIR /app

COPY service-vehicle/ service-vehicle/
COPY common/ common/

RUN pip install --no-cache-dir -r service-vehicle/requirements.txt

# Run the script
CMD ["python", "-m", "service-vehicle.main"]