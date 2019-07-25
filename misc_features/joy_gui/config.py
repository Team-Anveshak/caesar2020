from flask_compress import Compress
COMPRESS_MIMETYPES = ['text/html', 'text/css', 'text/xml', 'application/json', 'application/javascript']
COMPRESS_LEVEL = 6
COMPRESS_MIN_SIZE = 500
CACHE_TYPE = 'simple'

def configure_app(app):
    Compress(app)

 
