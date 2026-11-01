"""Procedural textures for the warehouse club world (all original, no brands).

Each texture is written once to <out>/textures/<name>.jpg or .png and shared by every glTF file.
make_all(out_dir) returns {name: filename}.
"""
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

RNG = np.random.default_rng(2026)
FONT_DIR = Path("/usr/share/fonts/truetype")
F_BOLD = str(FONT_DIR / "google-fonts/Poppins-Bold.ttf")
F_MED = str(FONT_DIR / "google-fonts/Poppins-Medium.ttf")
F_COND = str(FONT_DIR / "dejavu/DejaVuSansCondensed-Bold.ttf")


def font(path, size):
    """The fonts used here, or DejaVu (installed on most Linux systems), or Pillow's own font."""
    for cand in (path, "DejaVuSans-Bold.ttf", "/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf"):
        try:
            return ImageFont.truetype(cand, size)
        except OSError:
            continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:
        return ImageFont.load_default()


def noise(h, w, scale, rng=RNG, octaves=4, persistence=0.5):
    """Fractal value noise in [0, 1], tileable (wraps at the edges)."""
    out = np.zeros((h, w), np.float32)
    amp, total = 1.0, 0.0
    for o in range(octaves):
        gh, gw = max(2, int(h / scale * 2 ** o)), max(2, int(w / scale * 2 ** o))
        grid = rng.random((gh, gw)).astype(np.float32)
        grid = np.pad(grid, ((0, 1), (0, 1)), mode="wrap")
        img = Image.fromarray((grid * 255).astype(np.uint8)).resize(
            (int(w * (gw + 1) / gw), int(h * (gh + 1) / gh)), Image.BICUBIC)
        a = np.asarray(img, np.float32)[:h, :w] / 255.0
        out += amp * a
        total += amp
        amp *= persistence
    out /= total
    return (out - out.min()) / (out.max() - out.min() + 1e-6)


def to_img(arr):
    return Image.fromarray(np.clip(arr, 0, 255).astype(np.uint8))


def tint(gray, c0, c1):
    """Map a 0..1 field between two RGB colours."""
    c0, c1 = np.array(c0, np.float32), np.array(c1, np.float32)
    return gray[..., None] * (c1 - c0) + c0


# ------------------------------------------------------------------------------------------------
def concrete_floor(n=2048):
    """Polished concrete, one texture = 6 x 6 m, saw-cut joints on the tile edges."""
    base = noise(n, n, 700, octaves=5)
    mid = noise(n, n, 90, octaves=3)
    fine = RNG.random((n, n)).astype(np.float32)
    g = 0.55 * base + 0.30 * mid + 0.15 * fine
    img = tint(g, (150, 147, 140), (196, 193, 186))
    # aggregate speckles
    spk = RNG.random((n, n)) > 0.9965
    img[spk] = img[spk] * 0.78
    spk2 = RNG.random((n, n)) > 0.9975
    img[spk2] = np.minimum(img[spk2] * 1.12, 255)
    # faint trowel swirls
    yy, xx = np.mgrid[0:n, 0:n].astype(np.float32)
    for _ in range(9):
        cx, cy, r = RNG.random() * n, RNG.random() * n, 120 + RNG.random() * 260
        d = np.abs(np.hypot(xx - cx, yy - cy) - r)
        img -= (np.exp(-(d / 22) ** 2) * 2.2)[..., None]
    # a couple of stains
    for _ in range(3):
        cx, cy, r = RNG.random() * n, RNG.random() * n, 60 + RNG.random() * 140
        img -= (np.exp(-((xx - cx) ** 2 + (yy - cy) ** 2) / (2 * r * r)) * 10)[..., None]
    im = to_img(img)
    d = ImageDraw.Draw(im)
    j = 5
    for off in (0, n - 1):
        d.line([(0, off), (n, off)], fill=(92, 90, 86), width=j)
        d.line([(off, 0), (off, n)], fill=(92, 90, 86), width=j)
    return im.filter(ImageFilter.GaussianBlur(0.6))


def wall_panel(w=512, h=1024):
    """Insulated metal wall panel, 1 m wide x 2 m tall, light grey with ribs."""
    g = noise(h, w, 300, octaves=3) * 0.35 + 0.65
    img = tint(g, (205, 208, 208), (226, 228, 227))
    im = to_img(img)
    d = ImageDraw.Draw(im)
    for x in range(0, w, w // 4):
        d.line([(x, 0), (x, h)], fill=(186, 189, 190), width=3)
        d.line([(x + 4, 0), (x + 4, h)], fill=(236, 237, 236), width=2)
    d.line([(0, 0), (w, 0)], fill=(170, 172, 173), width=3)
    return im


def cmu_block(w=1024, h=512):
    """Painted concrete block, 2.4 m x 1.2 m: 400 x 200 mm units."""
    g = noise(h, w, 60, octaves=3) * 0.5 + 0.5
    img = tint(g, (172, 175, 176), (196, 198, 198))
    im = to_img(img)
    d = ImageDraw.Draw(im)
    bw, bh = w / 6, h / 6
    for r in range(7):
        y = int(r * bh)
        d.line([(0, y), (w, y)], fill=(146, 148, 149), width=4)
        shift = (bw / 2) if r % 2 else 0
        for c in range(8):
            x = int(c * bw + shift) % w
            d.line([(x, y), (x, int(y + bh))], fill=(146, 148, 149), width=4)
    return im


def painted_steel(color, w=256, h=256, rough=18):
    g = noise(h, w, 80, octaves=2)
    c = np.array(color, np.float32)
    img = c + (g[..., None] - 0.5) * rough
    return to_img(img)


def upright_face(color=(20, 72, 150), w=96, h=768):
    """Rack upright face, 76 mm x 0.6 m: blue with teardrop slot pairs every 50 mm."""
    im = painted_steel(color, w, h)
    d = ImageDraw.Draw(im)
    pitch = h / 12
    for k in range(12):
        y = k * pitch + pitch * 0.2
        for x in (w * 0.28, w * 0.72):
            d.ellipse([x - 7, y, x + 7, y + 12], fill=(12, 20, 32))
            d.polygon([(x - 5, y + 10), (x + 5, y + 10), (x, y + 26)], fill=(12, 20, 32))
    return im


def beam_face(w=512, h=64):
    """Orange step beam, 2.7 m x 0.11 m, with a small load-capacity sticker."""
    im = painted_steel((236, 96, 30), w, h, rough=14)
    d = ImageDraw.Draw(im)
    d.rectangle([w * 0.46, h * 0.25, w * 0.54, h * 0.75], fill=(245, 245, 240))
    d.rectangle([w * 0.465, h * 0.3, w * 0.535, h * 0.4], fill=(30, 30, 30))
    d.line([(0, 2), (w, 2)], fill=(250, 150, 90), width=2)
    return im


def wire_deck(n=512):
    """Wire mesh deck seen from above, one tile = 0.3 x 0.3 m (50 mm grid)."""
    im = Image.new("RGB", (n, n), (58, 60, 62))
    d = ImageDraw.Draw(im)
    step = n / 6
    for k in range(7):
        v = int(k * step)
        d.line([(v, 0), (v, n)], fill=(160, 163, 166), width=5)
        d.line([(0, v), (n, v)], fill=(140, 143, 146), width=4)
    return im


def pallet_wood(w=512, h=128):
    """Pine pallet board, 1.0 m x 0.25 m."""
    g = noise(h, w, 30, octaves=3)
    grain = (np.sin(np.linspace(0, 40, w)[None, :] + g * 6) * 0.5 + 0.5)
    v = 0.55 * g + 0.45 * grain
    img = tint(v, (170, 132, 88), (214, 180, 132))
    im = to_img(img)
    d = ImageDraw.Draw(im)
    for _ in range(3):
        x, y = RNG.random() * w, RNG.random() * h
        d.ellipse([x - 6, y - 4, x + 6, y + 4], fill=(120, 86, 50))
    return im.filter(ImageFilter.GaussianBlur(0.5))


# ---- carton designs (generic products) -------------------------------------------------------------
PRODUCTS = [
    # word, sub, print colour, band colour, board ("kraft" or "white"), icon
    ("PAPER TOWELS", "12 ROLLS · 2-PLY", (24, 92, 170), (24, 92, 170), "kraft", "rolls"),
    ("BATH TISSUE", "30 DOUBLE ROLLS", (40, 140, 90), (40, 140, 90), "kraft", "rolls"),
    ("SPRING WATER", "24 × 500 mL", (0, 120, 190), (130, 200, 240), "white", "drop"),
    ("SPARKLING WATER", "24 × 355 mL · LIME", (20, 150, 110), (190, 230, 90), "white", "bubbles"),
    ("ORANGE JUICE", "6 × 1.75 L", (230, 120, 20), (250, 190, 40), "white", "circle"),
    ("COFFEE BEANS", "DARK ROAST · 1 kg × 6", (90, 50, 30), (40, 30, 25), "kraft", "bean"),
    ("BASMATI RICE", "10 kg", (150, 40, 50), (220, 190, 110), "white", "grain"),
    ("ALL-PURPOSE FLOUR", "4 × 10 kg", (40, 70, 140), (240, 240, 235), "kraft", "wheat"),
    ("OLIVE OIL", "EXTRA VIRGIN · 2 × 3 L", (70, 100, 30), (200, 180, 60), "kraft", "leaf"),
    ("LAUNDRY DETERGENT", "HE · 4 × 4.4 L", (30, 60, 160), (250, 120, 40), "white", "wave"),
    ("DISH TABLETS", "160 COUNT", (20, 130, 150), (240, 240, 245), "white", "bubbles"),
    ("TRASH BAGS", "200 × 50 L", (40, 40, 45), (240, 200, 30), "kraft", "bars"),
    ("BREAKFAST CEREAL", "2 × 1.2 kg", (200, 50, 40), (250, 210, 60), "white", "circle"),
    ("TORTILLA CHIPS", "4 × 1 kg", (210, 110, 20), (60, 120, 50), "white", "triangle"),
    ("DOG FOOD", "CHICKEN & RICE · 18 kg", (120, 70, 30), (60, 110, 60), "kraft", "paw"),
    ("DICED TOMATOES", "12 × 796 mL", (190, 30, 30), (60, 130, 50), "kraft", "circle"),
    ("PAPER PLATES", "300 COUNT · 26 cm", (80, 80, 160), (230, 230, 240), "kraft", "circle"),
    ("MIXED NUTS", "UNSALTED · 1.1 kg × 6", (150, 90, 30), (230, 160, 60), "kraft", "grain"),
]


def carton(i, w=512, h=384):
    word, sub, pc, bc, board, icon = PRODUCTS[i]
    rng = np.random.default_rng(100 + i)
    if board == "kraft":
        g = noise(h, w, 50, rng=rng, octaves=3)
        fib = rng.random((h, w)).astype(np.float32)
        img = tint(0.7 * g + 0.3 * fib, (160, 122, 80), (196, 158, 110))
    else:
        g = noise(h, w, 80, rng=rng, octaves=2)
        img = tint(g, (232, 232, 228), (248, 248, 245))
    im = to_img(img)
    d = ImageDraw.Draw(im)
    band_h = int(h * 0.2)
    d.rectangle([0, h - band_h, w, h], fill=bc)
    d.rectangle([0, 0, w, int(h * 0.05)], fill=bc)
    f1 = font(F_BOLD, 44 if len(word) < 12 else 34 if len(word) < 16 else 28)
    tw = d.textlength(word, font=f1)
    d.text(((w - tw) / 2, h * 0.20), word, font=f1, fill=pc)
    f2 = font(F_MED, 24)
    tw2 = d.textlength(sub, font=f2)
    d.text(((w - tw2) / 2, h * 0.20 + 58), sub, font=f2, fill=pc)
    # icon
    cx, cy, r = w * 0.5, h * 0.60, 26
    ic = pc
    if icon == "rolls":
        for k in (-1, 0, 1):
            d.ellipse([cx + k * 60 - r, cy - r, cx + k * 60 + r, cy + r], outline=ic, width=6)
            d.ellipse([cx + k * 60 - 8, cy - 8, cx + k * 60 + 8, cy + 8], fill=ic)
    elif icon == "drop":
        d.polygon([(cx, cy - 36), (cx - 24, cy + 6), (cx + 24, cy + 6)], fill=ic)
        d.ellipse([cx - 24, cy - 14, cx + 24, cy + 30], fill=ic)
    elif icon == "bubbles":
        for k, (dx, dy, rr) in enumerate([(-40, 10, 14), (0, -8, 22), (40, 12, 12), (22, 28, 8)]):
            d.ellipse([cx + dx - rr, cy + dy - rr, cx + dx + rr, cy + dy + rr], outline=ic, width=5)
    elif icon == "triangle":
        d.polygon([(cx - 34, cy + 24), (cx + 34, cy + 24), (cx, cy - 34)], fill=ic)
    elif icon == "bars":
        for k in range(5):
            d.rectangle([cx - 60 + k * 26, cy - 24, cx - 48 + k * 26, cy + 24], fill=ic)
    elif icon == "paw":
        d.ellipse([cx - 22, cy - 6, cx + 22, cy + 30], fill=ic)
        for dx, dy in ((-30, -20), (-10, -34), (10, -34), (30, -20)):
            d.ellipse([cx + dx - 9, cy + dy - 9, cx + dx + 9, cy + dy + 9], fill=ic)
    elif icon == "wave":
        for k in range(3):
            d.arc([cx - 60, cy - 30 + k * 16, cx + 60, cy + 10 + k * 16], 200, 340, fill=ic, width=6)
    elif icon in ("leaf", "wheat", "grain", "bean"):
        for k in (-1, 0, 1):
            d.ellipse([cx + k * 34 - 12, cy - 26, cx + k * 34 + 12, cy + 26], fill=ic)
    else:
        d.ellipse([cx - r, cy - r, cx + r, cy + r], fill=ic)
    # handling marks and a barcode
    d.polygon([(26, 40), (38, 22), (50, 40)], fill=(40, 40, 40))
    d.rectangle([35, 40, 41, 60], fill=(40, 40, 40))
    d.polygon([(56, 40), (68, 22), (80, 40)], fill=(40, 40, 40))
    d.rectangle([65, 40, 71, 60], fill=(40, 40, 40))
    x = w - 150
    d.rectangle([x - 6, 24, w - 22, 78], fill=(250, 250, 250))
    while x < w - 30:
        bw_ = int(rng.integers(1, 5))
        d.rectangle([x, 30, x + bw_, 72], fill=(20, 20, 20))
        x += bw_ + int(rng.integers(1, 4))
    # carton edge shading (so a stack reads as separate boxes)
    arr = np.asarray(im, np.float32)
    ramp = np.ones((h, w), np.float32)
    e = 10
    ramp[:e, :] *= np.linspace(0.62, 1, e)[:, None]
    ramp[-e:, :] *= np.linspace(1, 0.66, e)[:, None]
    ramp[:, :e] *= np.linspace(0.66, 1, e)[None, :]
    ramp[:, -e:] *= np.linspace(1, 0.62, e)[None, :]
    return to_img(arr * ramp[..., None])


def stretch_wrap(i, w=512, h=512):
    """Overstock pallet under stretch film: the carton print, washed out, with film highlights."""
    base = np.asarray(carton(i % len(PRODUCTS), w, h), np.float32)
    film = noise(h, w, 120, rng=np.random.default_rng(300 + i), octaves=3)
    streak = (np.sin(np.linspace(0, 28, h)[:, None] + film * 3) * 0.5 + 0.5)
    white = np.array([236, 238, 240], np.float32)
    k = (0.45 + 0.25 * streak)[..., None]
    return to_img(base * (1 - k) + white * k)


def water_bundle(w=512, h=384):
    """Shrink-wrapped case of bottles: blue caps and bottle shoulders under clear film."""
    im = Image.new("RGB", (w, h), (170, 200, 222))
    d = ImageDraw.Draw(im)
    for r in range(3):
        for c in range(6):
            cx, cy = 42 + c * 86, 60 + r * 120
            d.ellipse([cx - 34, cy - 10, cx + 34, cy + 100], fill=(120, 170, 210))
            d.rectangle([cx - 12, cy - 24, cx + 12, cy - 6], fill=(20, 80, 180))
            d.ellipse([cx - 22, cy + 8, cx - 6, cy + 60], fill=(215, 235, 248))
    d.rectangle([0, h * 0.55, w, h * 0.72], fill=(0, 110, 185))
    f = font(F_BOLD, 30)
    d.text((24, h * 0.57), "SPRING WATER  ·  24 × 500 mL", font=f, fill=(255, 255, 255))
    return im.filter(ImageFilter.GaussianBlur(0.8))


def towel_bundle(w=512, h=512):
    """Paper towel bundle: white rolls under printed film."""
    im = Image.new("RGB", (w, h), (238, 240, 242))
    d = ImageDraw.Draw(im)
    for r in range(2):
        for c in range(3):
            x0, y0 = c * w / 3, r * h / 2
            d.rectangle([x0 + 6, y0 + 6, x0 + w / 3 - 6, y0 + h / 2 - 6], fill=(250, 250, 250), outline=(215, 218, 222), width=3)
    d.rectangle([0, h * 0.4, w, h * 0.6], fill=(24, 92, 170))
    f = font(F_BOLD, 46)
    d.text((30, h * 0.415), "PAPER TOWELS · 12", font=f, fill=(255, 255, 255))
    return im


def rubber_mat(n=512):
    im = to_img(tint(noise(n, n, 40, octaves=2), (38, 38, 40), (58, 58, 60)))
    d = ImageDraw.Draw(im)
    for y in range(0, n, 16):
        d.line([(0, y), (n, y)], fill=(28, 28, 30), width=5)
    return im


def asphalt(n=1024):
    g = 0.6 * noise(n, n, 200, octaves=4) + 0.4 * RNG.random((n, n)).astype(np.float32)
    return to_img(tint(g, (48, 49, 51), (86, 87, 88)))


def stainless(w=512, h=256):
    streak = np.repeat(RNG.random((h, 1)).astype(np.float32), w, axis=1)
    streak = np.asarray(to_img(streak * 255).filter(ImageFilter.GaussianBlur((6, 0))), np.float32) / 255
    g = 0.7 * streak + 0.3 * noise(h, w, 100, octaves=2)
    return to_img(tint(g, (168, 172, 176), (208, 212, 215)))


def conveyor_belt(w=256, h=512):
    im = to_img(tint(noise(h, w, 30, octaves=2), (22, 22, 24), (40, 40, 43)))
    d = ImageDraw.Draw(im)
    for y in range(0, h, 12):
        d.line([(0, y), (w, y)], fill=(16, 16, 18), width=3)
    return im


def wood_laminate(w=1024, h=512):
    g = noise(h, w, 60, octaves=3)
    grain = np.sin(np.linspace(0, 60, h)[:, None] + g * 5) * 0.5 + 0.5
    return to_img(tint(0.6 * grain + 0.4 * g, (176, 136, 96), (210, 174, 132)))


def fridge_interior(w=1024, h=1024):
    """Back of a glass-door fridge: 5 lit shelves of dairy and drinks (one door = 0.76 x 2.1 m)."""
    im = Image.new("RGB", (w, h), (232, 236, 240))
    d = ImageDraw.Draw(im)
    rng = np.random.default_rng(7)
    colours = [(250, 250, 250), (240, 244, 250), (60, 120, 200), (230, 60, 50), (250, 200, 40), (90, 170, 80),
               (250, 250, 235), (200, 90, 160)]
    for s in range(5):
        y1 = int((s + 1) * h / 5)
        d.rectangle([0, y1 - 14, w, y1], fill=(150, 156, 162))
        x = 6
        while x < w - 40:
            pw = int(rng.integers(44, 90)); ph = int(rng.integers(90, 170))
            c = colours[int(rng.integers(len(colours)))]
            d.rounded_rectangle([x, y1 - 14 - ph, x + pw, y1 - 16], radius=10, fill=c, outline=(180, 184, 188))
            if rng.random() < 0.6:
                d.rectangle([x + 6, y1 - 14 - ph * 0.6, x + pw - 6, y1 - 14 - ph * 0.35], fill=(40, 90, 170))
            x += pw + int(rng.integers(3, 10))
    return im.filter(ImageFilter.GaussianBlur(1.2))


def freezer_contents(w=1024, h=256):
    im = Image.new("RGB", (w, h), (230, 236, 242))
    d = ImageDraw.Draw(im)
    rng = np.random.default_rng(11)
    x = 0
    while x < w:
        bw = int(rng.integers(70, 140))
        c = [(40, 90, 170), (220, 60, 40), (240, 240, 245), (40, 140, 90), (250, 190, 40)][int(rng.integers(5))]
        d.rectangle([x + 3, 20, x + bw - 3, h - 10], fill=c, outline=(200, 205, 210), width=3)
        d.rectangle([x + 10, 60, x + bw - 10, 100], fill=(250, 250, 250))
        x += bw
    return im


def ceiling_deck(w=512, h=512):
    """Corrugated steel roof deck seen from below, 1.2 m tile, ribs every 150 mm."""
    im = Image.new("RGB", (w, h), (226, 228, 229))
    d = ImageDraw.Draw(im)
    for k in range(8):
        x = int(k * w / 8)
        d.rectangle([x, 0, x + int(w / 16), h], fill=(206, 208, 210))
        d.line([(x, 0), (x, h)], fill=(180, 182, 185), width=2)
    return im


def dock_door(w=512, h=512):
    """Sectional overhead door panels with one row of windows (3.5 x 3.5 m)."""
    im = Image.new("RGB", (w, h), (214, 216, 214))
    d = ImageDraw.Draw(im)
    for k in range(1, 7):
        y = int(k * h / 7)
        d.line([(0, y), (w, y)], fill=(150, 152, 150), width=5)
    for c in range(4):
        x = 40 + c * 120
        d.rectangle([x, int(h * 2 / 7) + 20, x + 80, int(h * 3 / 7) - 20], fill=(70, 90, 110))
    return im


def hazard_stripes(w=512, h=128):
    im = Image.new("RGB", (w, h), (250, 200, 20))
    d = ImageDraw.Draw(im)
    for x in range(-h, w + h, 64):
        d.polygon([(x, h), (x + 32, h), (x + 32 + h, 0), (x + h, 0)], fill=(25, 25, 25))
    return im


def sign(text, w=1024, h=384, bg=(18, 52, 110), fg=(255, 255, 255), size=190, sub=None):
    im = Image.new("RGB", (w, h), bg)
    d = ImageDraw.Draw(im)
    d.rectangle([10, 10, w - 10, h - 10], outline=(255, 255, 255), width=8)
    f = font(F_BOLD, size)
    bbox = d.textbbox((0, 0), text, font=f)
    tw, th = bbox[2] - bbox[0], bbox[3] - bbox[1]
    y = (h - th) / 2 - bbox[1] - (30 if sub else 0)
    d.text(((w - tw) / 2 - bbox[0], y), text, font=f, fill=fg)
    if sub:
        f2 = font(F_MED, 60)
        tw2 = d.textlength(sub, font=f2)
        d.text(((w - tw2) / 2, h - 110), sub, font=f2, fill=fg)
    return im


def apriltag(tag_id=0, px=600):
    import cv2
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    if hasattr(cv2.aruco, "generateImageMarker"):
        inner = cv2.aruco.generateImageMarker(dic, tag_id, px)
    else:                                                   # OpenCV 4.6 and older
        inner = cv2.aruco.drawMarker(dic, tag_id, px)
    pad = px // 8
    img = np.full((px + 2 * pad, px + 2 * pad), 255, np.uint8)
    img[pad:pad + px, pad:pad + px] = inner
    return Image.fromarray(img).convert("RGB")


def clothing_stack(w=512, h=512):
    """Folded clothes stacks seen from the side (0.5 x 0.5 m)."""
    im = Image.new("RGB", (w, h), (240, 240, 240))
    d = ImageDraw.Draw(im)
    rng = np.random.default_rng(21)
    y = h
    cols = [(40, 60, 110), (180, 40, 50), (90, 90, 95), (220, 220, 215), (60, 110, 70), (30, 30, 35), (150, 120, 90)]
    while y > 0:
        t = int(rng.integers(24, 40))
        c = cols[int(rng.integers(len(cols)))]
        d.rounded_rectangle([8, y - t, w - 8, y - 2], radius=12, fill=c)
        y -= t
    return im


def electronics_box(i, w=512, h=384):
    rng = np.random.default_rng(40 + i)
    im = Image.new("RGB", (w, h), (24, 26, 30) if i % 2 == 0 else (240, 242, 244))
    d = ImageDraw.Draw(im)
    fg = (240, 240, 240) if i % 2 == 0 else (30, 30, 34)
    # a product silhouette: screen, laptop, speaker, router
    kind = i % 4
    if kind == 0:
        d.rectangle([60, 70, w - 60, h - 110], fill=(40, 90, 170))
        d.rectangle([w / 2 - 20, h - 110, w / 2 + 20, h - 80], fill=(120, 120, 130))
    elif kind == 1:
        d.polygon([(90, h - 90), (w - 90, h - 90), (w - 60, h - 60), (60, h - 60)], fill=(150, 150, 160))
        d.rectangle([110, 80, w - 110, h - 95], fill=(60, 140, 200))
    elif kind == 2:
        d.rounded_rectangle([w / 2 - 70, 60, w / 2 + 70, h - 60], radius=30, fill=(90, 90, 100))
        d.ellipse([w / 2 - 40, 110, w / 2 + 40, 190], fill=(40, 40, 45))
    else:
        d.rounded_rectangle([90, h / 2 - 30, w - 90, h / 2 + 50], radius=16, fill=(210, 210, 215))
        for k in range(3):
            d.line([(150 + k * 90, h / 2 - 30), (130 + k * 110, 70)], fill=(120, 120, 130), width=10)
    f = font(F_BOLD, 34)
    names = ["55\" 4K DISPLAY", "14\" LAPTOP", "SMART SPEAKER", "MESH WI-FI 3-PACK"]
    d.text((30, 18), names[kind], font=f, fill=fg)
    return im


def wet_floor_panel(w=384, h=768):
    im = Image.new("RGB", (w, h), (250, 204, 24))
    d = ImageDraw.Draw(im)
    d.polygon([(w / 2, 90), (w - 50, 330), (50, 330)], fill=(20, 20, 20))
    d.polygon([(w / 2, 130), (w - 85, 308), (85, 308)], fill=(250, 204, 24))
    d.rectangle([w / 2 - 12, 170, w / 2 + 12, 260], fill=(20, 20, 20))
    d.ellipse([w / 2 - 13, 272, w / 2 + 13, 298], fill=(20, 20, 20))
    f = font(F_BOLD, 64)
    for k, t in enumerate(("CAUTION", "WET", "FLOOR")):
        tw = d.textlength(t, font=f)
        d.text(((w - tw) / 2, 380 + k * 100), t, font=f, fill=(20, 20, 20))
    return im


def pallet_front(w=512, h=74):
    """Pallet face with the fork openings, 1.0 m x 0.144 m (for pallets drawn as one box)."""
    im = pallet_wood(w, h)
    d = ImageDraw.Draw(im)
    top, bot = int(h * 0.153), int(h * 0.847)
    for x0, x1 in ((0.1, 0.45), (0.55, 0.9)):
        d.rectangle([int(x0 * w), top, int(x1 * w), bot], fill=(34, 28, 22))
    d.line([(0, top), (w, top)], fill=(120, 90, 60), width=2)
    d.line([(0, bot), (w, bot)], fill=(120, 90, 60), width=2)
    return im


def pallet_side(w=512, h=62):
    """Pallet side (the stringer), 1.2 m x 0.144 m."""
    im = pallet_wood(w, h)
    d = ImageDraw.Draw(im)
    for y in (int(h * 0.153), int(h * 0.847)):
        d.line([(0, y), (w, y)], fill=(118, 88, 58), width=2)
    return im


def office_tile(n=512):
    """Vinyl composition tile, 0.3 m squares, one texture = 1.2 x 1.2 m."""
    g = 0.6 * noise(n, n, 40, octaves=3) + 0.4 * RNG.random((n, n)).astype(np.float32)
    img = tint(g, (190, 192, 190), (214, 216, 213))
    step = n // 4
    for r in range(4):
        for c in range(4):
            if (r + c) % 3 == 0:
                img[r * step:(r + 1) * step, c * step:(c + 1) * step] *= 0.93
    im = to_img(img)
    d = ImageDraw.Draw(im)
    for k in range(5):
        v = min(n - 1, k * step)
        d.line([(v, 0), (v, n)], fill=(168, 170, 168), width=2)
        d.line([(0, v), (n, v)], fill=(168, 170, 168), width=2)
    return im


def ceiling_tile(n=512):
    """Suspended ceiling, 0.6 m tiles in a white T-bar grid, one texture = 1.2 x 1.2 m."""
    g = 0.5 * noise(n, n, 20, octaves=2) + 0.5 * RNG.random((n, n)).astype(np.float32)
    im = to_img(tint(g, (224, 224, 220), (240, 240, 236)))
    d = ImageDraw.Draw(im)
    for v in (0, n // 2, n - 1):
        d.line([(v, 0), (v, n)], fill=(250, 250, 250), width=7)
        d.line([(0, v), (n, v)], fill=(250, 250, 250), width=7)
    return im


def books(w=512, h=384):
    """Stacks of hardcover books seen from the side (0.5 x 0.3 m)."""
    rng = np.random.default_rng(31)
    im = Image.new("RGB", (w, h), (236, 234, 228))
    d = ImageDraw.Draw(im)
    cols = [(150, 30, 40), (30, 60, 110), (240, 236, 220), (40, 90, 60), (20, 20, 24), (200, 150, 40), (90, 60, 120)]
    y = h
    while y > 0:
        t = int(rng.integers(18, 34))
        c = cols[int(rng.integers(len(cols)))]
        x0 = int(rng.integers(0, 12))
        d.rectangle([x0, y - t, w - int(rng.integers(0, 12)), y - 2], fill=c)
        d.rectangle([x0 + 20, y - t + 5, x0 + 60, y - 7], fill=(230, 200, 120))
        y -= t
    return im


def menu_board(w=1536, h=512):
    """Food court menu board: generic items and prices on a dark board."""
    im = Image.new("RGB", (w, h), (26, 30, 36))
    d = ImageDraw.Draw(im)
    d.rectangle([0, 0, w, 86], fill=(180, 30, 36))
    f0, f1, f2 = font(F_BOLD, 60), font(F_BOLD, 50), font(F_MED, 44)
    d.text((40, 8), "FOOD COURT", font=f0, fill=(255, 255, 255))
    items = [("HOT DOG & DRINK", "2.00"), ("CHEESE PIZZA SLICE", "2.50"), ("CAESAR SALAD", "4.50"),
             ("BERRY SMOOTHIE", "3.00"), ("SOFT-SERVE CONE", "1.50"), ("CHURRO", "1.75")]
    for k, (name, price) in enumerate(items):
        col, row = k // 3, k % 3
        x, y = 50 + col * (w // 2), 125 + row * 125
        d.text((x, y), name, font=f1, fill=(250, 250, 245))
        tw = d.textlength(price, font=f1)
        d.text((x + w // 2 - 110 - tw, y), price, font=f1, fill=(250, 200, 40))
        d.line([(x, y + 78), (x + w // 2 - 110, y + 78)], fill=(70, 76, 84), width=3)
    return im


def service_case(kind, w=1024, h=256):
    """What shows through the glass of a service case: meat trays or bakery (one texture = 2 m of case)."""
    rng = np.random.default_rng(51 if kind == "meat" else 52)
    im = Image.new("RGB", (w, h), (236, 238, 240))
    d = ImageDraw.Draw(im)
    for row in range(2):
        y0 = 30 + row * 110
        x = 10
        while x < w - 60:
            tw = int(rng.integers(70, 120))
            if kind == "meat":
                c = [(170, 40, 50), (190, 70, 80), (215, 150, 140), (150, 30, 40), (230, 200, 170)][int(rng.integers(5))]
                d.rectangle([x, y0, x + tw, y0 + 80], fill=(30, 30, 34))
                d.ellipse([x + 8, y0 + 10, x + tw - 8, y0 + 72], fill=c)
            else:
                c = [(200, 140, 70), (170, 110, 50), (230, 190, 120), (120, 70, 40), (240, 220, 190)][int(rng.integers(5))]
                d.rounded_rectangle([x + 4, y0 + 20, x + tw - 4, y0 + 78], radius=24, fill=c)
                d.line([(x + 20, y0 + 36), (x + tw - 20, y0 + 36)], fill=(150, 100, 50), width=4)
            x += tw + int(rng.integers(6, 14))
    return im.filter(ImageFilter.GaussianBlur(0.8))


def trailer_side(w=1024, h=256):
    """Plain white semi-trailer side (8 m x 2 m) with vertical posts."""
    g = noise(h, w, 200, octaves=2) * 0.3 + 0.7
    im = to_img(tint(g, (214, 216, 216), (236, 237, 236)))
    d = ImageDraw.Draw(im)
    for k in range(17):
        x = int(k * w / 16)
        d.line([(x, 0), (x, h)], fill=(190, 192, 192), width=6)
    d.rectangle([0, h - 14, w, h], fill=(170, 172, 174))
    return im


DEPARTMENTS = {"grocery": "GROCERY", "household": "HOUSEHOLD", "beverages": "BEVERAGES", "paper": "PAPER GOODS",
               "electronics": "ELECTRONICS", "fresh": "FRESH", "frozen": "FROZEN", "bakery": "BAKERY",
               "pet": "PET SUPPLIES", "snacks": "SNACKS", "food_court": "FOOD COURT", "exit": "EXIT",
               "entrance": "WELCOME", "service": "MEMBER SERVICES", "clothing": "CLOTHING", "books": "BOOKS",
               "dairy": "DAIRY", "meat": "MEAT & DELI", "seasonal": "SEASONAL"}


def make_all(out_dir):
    out = Path(out_dir) / "textures"
    out.mkdir(parents=True, exist_ok=True)
    tex = {}

    def save(name, im, fmt="jpg", q=88):
        fn = f"{name}.{fmt}"
        if fmt == "jpg":
            im.convert("RGB").save(out / fn, quality=q, optimize=True)
        else:
            im.save(out / fn, optimize=True)
        tex[name] = fn

    save("concrete_floor", concrete_floor(), q=85)
    save("wall_panel", wall_panel())
    save("cmu_block", cmu_block())
    save("upright_blue", upright_face())
    save("beam_orange", beam_face())
    save("steel_orange", painted_steel((236, 96, 30)))
    save("steel_blue", painted_steel((20, 72, 150)))
    save("steel_grey", painted_steel((120, 124, 128)))
    save("steel_yellow", painted_steel((244, 190, 20)))
    save("wire_deck", wire_deck())
    save("pallet_wood", pallet_wood())
    for i in range(len(PRODUCTS)):
        save(f"carton_{i:02d}", carton(i))
    for i in range(6):
        save(f"wrap_{i:02d}", stretch_wrap(i * 3 + 1))
    save("water_bundle", water_bundle())
    save("towel_bundle", towel_bundle())
    save("rubber_mat", rubber_mat())
    save("asphalt", asphalt())
    save("stainless", stainless())
    save("belt", conveyor_belt())
    save("wood_laminate", wood_laminate())
    save("fridge_interior", fridge_interior())
    save("freezer_contents", freezer_contents())
    save("ceiling_deck", ceiling_deck())
    save("dock_door", dock_door())
    save("hazard_stripes", hazard_stripes())
    save("clothing", clothing_stack())
    for i in range(4):
        save(f"electronics_{i}", electronics_box(i))
    save("apriltag_0", apriltag(0), fmt="png")
    save("wet_floor_panel", wet_floor_panel(), fmt="png")
    for k in range(1, 31):
        save(f"aisle_{k:02d}", sign(f"{k}", w=512, h=640, size=300, sub="AISLE"))
    for key, text in DEPARTMENTS.items():
        bg = (180, 30, 36) if key in ("exit",) else (18, 52, 110)
        save(f"dept_{key}", sign(text, w=2048, h=384, size=170, bg=bg))
    save("dept_receiving", sign("RECEIVING", w=2048, h=384, size=150, bg=(180, 30, 36), sub="EMPLOYEES ONLY"))
    save("pallet_front", pallet_front())
    save("pallet_side", pallet_side())
    save("office_tile", office_tile())
    save("ceiling_tile", ceiling_tile())
    save("books", books())
    save("menu_board", menu_board())
    save("case_meat", service_case("meat"))
    save("case_bakery", service_case("bakery"))
    save("trailer_side", trailer_side())
    for k in range(1, 15):
        save(f"lane_{k:02d}", sign(f"{k}", w=384, h=384, size=230, bg=(18, 52, 110)))
    save("sign_office", sign("OFFICE", w=1024, h=256, size=130, bg=(60, 66, 74)))
    save("sign_dock", sign("DOCK EDGE", w=1024, h=256, size=110, bg=(180, 30, 36), sub=None))
    return tex


if __name__ == "__main__":
    import sys
    t = make_all(sys.argv[1] if len(sys.argv) > 1 else ".")
    print(len(t), "textures")
