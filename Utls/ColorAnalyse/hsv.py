import cv2
import os
import json
from tkinter import Tk, Button, Label, filedialog, messagebox
from matplotlib import pyplot as plt

roi_file_with_lines = "ROICords/roi_with_lines.json"
roi_file_without_lines = "ROICords/roi_without_lines.json"

def select_multiple_rois(image, label):
    display_image = image.copy()
    rois = []

    while True:
        resized_image = cv2.resize(display_image, (800, 600))
        roi = cv2.selectROI(f"Select {label} ROI (trykk Esc eller 'c' for å avslutte)", resized_image)
        if roi == (0, 0, 0, 0):
            break

        scale_x = image.shape[1] / 800
        scale_y = image.shape[0] / 600
        x = int(round(roi[0] * scale_x))
        y = int(round(roi[1] * scale_y))
        w = int(round(roi[2] * scale_x))
        h = int(round(roi[3] * scale_y))

        if w > 0 and h > 0:
            cv2.rectangle(display_image, (x, y), (x + w, y + h), (255, 0, 0), 2)
            rois.append((x, y, w, h))

    cv2.destroyAllWindows()
    return rois

def save_rois_to_file(rois_dict, filename):
    with open(filename, "w") as f:
        json.dump(rois_dict, f)
    messagebox.showinfo("Lagring fullført", f"ROI-koordinater lagret til {filename}")

def load_rois_from_file(filename):
    if os.path.exists(filename):
        with open(filename, "r") as f:
            rois = json.load(f)
        return rois
    else:
        messagebox.showwarning("Feil", f"{filename} finnes ikke.")
    return {}

def load_images_from_folder(folder):
    images = []
    for filename in os.listdir(folder):
        filepath = os.path.join(folder, filename)
        if filename.endswith(".jpg") or filename.endswith(".png"):
            img = cv2.imread(filepath)
            if img is not None:
                images.append((filename, img))
    return images

def show_histograms(images, rois_dict, title):
    h_values, s_values, v_values = [], [], []
    r_values, g_values, b_values = [], [], []

    for filename, image in images:
        rois = rois_dict.get(filename, [])
        for (x, y, w, h) in rois:
            roi_cropped = image[y:y + h, x:x + w]
            if roi_cropped.size == 0:
                continue

            hsv_roi = cv2.cvtColor(roi_cropped, cv2.COLOR_BGR2HLS)
            h_values.extend(hsv_roi[:, :, 0].flatten())
            s_values.extend(hsv_roi[:, :, 1].flatten())
            v_values.extend(hsv_roi[:, :, 2].flatten())

            b_values.extend(roi_cropped[:, :, 0].flatten())
            g_values.extend(roi_cropped[:, :, 1].flatten())
            r_values.extend(roi_cropped[:, :, 2].flatten())

    plt.figure(figsize=(12, 8))
    plt.suptitle(title)

    plt.subplot(2, 3, 1)
    plt.hist(h_values, bins=30, color='r', alpha=0.7)
    plt.title('Hue')
    plt.xlabel('Value')
    plt.ylabel('Frequency')

    plt.subplot(2, 3, 2)
    plt.hist(s_values, bins=30, color='g', alpha=0.7)
    plt.title('Saturation')
    plt.xlabel('Value')
    plt.ylabel('Frequency')

    plt.subplot(2, 3, 3)
    plt.hist(v_values, bins=30, color='b', alpha=0.7)
    plt.title('Value')
    plt.xlabel('Value')
    plt.ylabel('Frequency')

    plt.subplot(2, 3, 4)
    plt.hist(r_values, bins=30, color='r', alpha=0.7)
    plt.title('Red')
    plt.xlabel('Value')
    plt.ylabel('Frequency')

    plt.subplot(2, 3, 5)
    plt.hist(g_values, bins=30, color='g', alpha=0.7)
    plt.title('Green')
    plt.xlabel('Value')
    plt.ylabel('Frequency')

    plt.subplot(2, 3, 6)
    plt.hist(b_values, bins=30, color='b', alpha=0.7)
    plt.title('Blue')
    plt.xlabel('Value')
    plt.ylabel('Frequency')

    plt.tight_layout()
    plt.show()

def display_images_with_rois(images, rois_dict, title):
    for filename, image in images:
        display_image = image.copy()
        rois = rois_dict.get(filename, [])
        for (x, y, w, h) in rois:
            cv2.rectangle(display_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

        resized_display_image = cv2.resize(display_image, (800, 600))
        cv2.imshow(f"{title} - {filename}", resized_display_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

def load_images():
    global images
    folder_path = filedialog.askdirectory(title="Velg mappen med bilder")
    if folder_path:
        images = load_images_from_folder(folder_path)
        messagebox.showinfo("Bilder lastet", f"{len(images)} bilder lastet fra {folder_path}")

def select_rois_with_lines():
    rois_with_lines = {}
    for filename, image in images:
        rois_with_lines[filename] = select_multiple_rois(image, "with lines")
    save_rois_to_file(rois_with_lines, roi_file_with_lines)

def select_rois_without_lines():
    rois_without_lines = {}
    for filename, image in images:
        rois_without_lines[filename] = select_multiple_rois(image, "without lines")
    save_rois_to_file(rois_without_lines, roi_file_without_lines)

def show_histogram_with_lines():
    rois_with_lines = load_rois_from_file(roi_file_with_lines)
    if rois_with_lines:
        show_histograms(images, rois_with_lines, "Histogrammer for områder med svarte linjer")

def show_histogram_without_lines():
    rois_without_lines = load_rois_from_file(roi_file_without_lines)
    if rois_without_lines:
        show_histograms(images, rois_without_lines, "Histogrammer for områder uten svarte linjer")

def show_images_with_lines():
    rois_with_lines = load_rois_from_file(roi_file_with_lines)
    if rois_with_lines:
        display_images_with_rois(images, rois_with_lines, "Områder med svarte linjer")

def show_images_without_lines():
    rois_without_lines = load_rois_from_file(roi_file_without_lines)
    if rois_without_lines:
        display_images_with_rois(images, rois_without_lines, "Områder uten svarte linjer")

root = Tk()
root.title("ROI Analysator")
root.geometry("300x400")

Button(root, text="Last inn bilder", command=load_images, width=25).pack(pady=10)
Button(root, text="Velg områder med svarte linjer", command=select_rois_with_lines, width=25).pack(pady=5)
Button(root, text="Velg områder uten svarte linjer", command=select_rois_without_lines, width=25).pack(pady=5)
Button(root, text="Vis histogrammer (svarte linjer)", command=show_histogram_with_lines, width=25).pack(pady=10)
Button(root, text="Vis histogrammer (uten svarte linjer)", command=show_histogram_without_lines, width=25).pack(pady=5)
Button(root, text="Vis bilder med svarte linjer", command=show_images_with_lines, width=25).pack(pady=10)
Button(root, text="Vis bilder uten svarte linjer", command=show_images_without_lines, width=25).pack(pady=5)
Button(root, text="Avslutt", command=root.quit, width=25).pack(pady=10)

root.mainloop()
