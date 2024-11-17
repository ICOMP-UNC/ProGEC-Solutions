import os
from sendgrid import SendGridAPIClient
from sendgrid.helpers.mail import Mail

# Configuración del correo
SENDGRID_API_KEY = os.environ.get("SENDGRID_API_KEY")  # Guardar la API key en variables de entorno
EMAIL_SENDER = "gabrielarrietacarrasco@gmail.com"  # Dirección desde donde se envía
EMAIL_RECEIVER = "primallea29@gmail.com"  # Correo temporal como receptor

def enviar_alerta(tipo, valor):
    """Envía una alerta usando SendGrid cuando un valor está fuera de rango."""
    mensaje = Mail(
        from_email=EMAIL_SENDER,
        to_emails=EMAIL_RECEIVER,
        subject=f"⚠️ Alerta: {tipo.capitalize()} fuera de rango seguro",
        plain_text_content=f"El valor de {tipo} es {valor}, lo cual está fuera del rango seguro."
    )
    try:
        sg = SendGridAPIClient(SENDGRID_API_KEY)
        response = sg.send(mensaje)
        print(f"Correo de alerta enviado: {mensaje}")
    except Exception as e:
        print(f"Error al enviar correo: {e}")