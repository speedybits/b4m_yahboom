ARG BUILD_FROM
FROM $BUILD_FROM

# Install Python and dependencies
RUN apk add --no-cache python3 py3-pip

WORKDIR /app

# Copy requirements and install dependencies
COPY requirements.txt .
RUN pip3 install --no-cache-dir -r requirements.txt

# Copy application and run script
COPY app.py .
COPY run.sh /
RUN chmod a+x /run.sh

# Expose port
EXPOSE 3000

# Run the application
CMD ["/run.sh"]
