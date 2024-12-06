import os
import qrcode
from pylibdmtx.pylibdmtx import encode
from PIL import Image

names = ["001", "002", "003", "004"]

cm_to_pixels = lambda cm: int(cm * 37.7952755906)
size_cm = 11.4
size_pixels = cm_to_pixels(size_cm)

qr_folder = "qr_codes"
dm_folder = "data_matrix_codes"

os.makedirs(qr_folder, exist_ok=True)
os.makedirs(dm_folder, exist_ok=True)

for name in names:
    qr = qrcode.QRCode(
        version=1,
        error_correction=qrcode.constants.ERROR_CORRECT_H,
        box_size=50,
        border=4,
    )
    qr.add_data(name)
    qr.make(fit=True)

    qr_img = qr.make_image(fill_color="black", back_color="white")
    qr_img = qr_img.resize((size_pixels, size_pixels))
    qr_path = os.path.join(qr_folder, f"{name.replace('-', '_')}_qr.png")
    qr_img.save(qr_path)
    print(f"QR-kode generert og lagret som: {qr_path}")

    dm_encoded = encode(name.encode())
    dm_img = Image.frombytes('RGB', (dm_encoded.width, dm_encoded.height), dm_encoded.pixels)
    dm_img = dm_img.resize((size_pixels, size_pixels))
    dm_path = os.path.join(dm_folder, f"{name.replace('-', '_')}_datamatrix.png")
    dm_img.save(dm_path)
    print(f"Data Matrix-kode generert og lagret som: {dm_path}")

print("Alle koder generert og lagret i respektive mapper.")
