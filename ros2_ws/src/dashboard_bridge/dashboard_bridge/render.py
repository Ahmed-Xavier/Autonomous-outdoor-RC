"""Pure Pillow drawing functions for the Kindle e-ink dashboard."""
import bisect
import io
import math
from functools import lru_cache

from PIL import Image, ImageDraw, ImageFont


W, H = 800, 600


@lru_cache(maxsize=None)
def font(size):
    for name in ("bahnschrift.ttf", "arialbd.ttf", "DejaVuSans-Bold.ttf"):
        try:
            loaded = ImageFont.truetype(name, size)
        except OSError:
            continue
        if name == "bahnschrift.ttf":
            try:
                loaded.set_variation_by_name("Bold")
            except Exception:
                pass
        return loaded
    return ImageFont.load_default()


def card(draw, box, inverted=False):
    """Draw a rounded card and return its foreground ink colour."""
    draw.rounded_rectangle(box, radius=14, outline=0, width=4,
                           fill=0 if inverted else 255)
    return 255 if inverted else 0


def smooth_closed(points, sigma):
    """Gaussian-smooth a closed loop of points, wrapping around the start."""
    if sigma <= 0:
        return list(points)
    count = len(points)
    radius = int(sigma * 3)
    weights = [math.exp(-(offset * offset) / (2 * sigma * sigma))
               for offset in range(-radius, radius + 1)]
    total = sum(weights)
    smoothed = []
    for index in range(count):
        x = sum(weights[offset + radius] * points[(index + offset) % count][0]
                for offset in range(-radius, radius + 1)) / total
        y = sum(weights[offset + radius] * points[(index + offset) % count][1]
                for offset in range(-radius, radius + 1)) / total
        smoothed.append((x, y))
    return smoothed


def fit_track(raw_points, map_box, smoothing_sigma):
    """Smooth and fit raw pixel coordinates into map_box; return points and distances."""
    points = smooth_closed([tuple(point) for point in raw_points], smoothing_sigma)
    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    x0, x1, y0, y1 = min(xs), max(xs), min(ys), max(ys)
    box_x0, box_y0, box_x1, box_y1 = map_box
    scale = min((box_x1 - box_x0) / (x1 - x0),
                (box_y1 - box_y0) / (y1 - y0))
    offset_x = box_x0 + ((box_x1 - box_x0) - scale * (x1 - x0)) / 2 - scale * x0
    offset_y = box_y0 + ((box_y1 - box_y0) - scale * (y1 - y0)) / 2 - scale * y0
    fitted = [(offset_x + scale * x, offset_y + scale * y) for x, y in points]
    fitted.append(fitted[0])
    distances = [0.0]
    for start, end in zip(fitted, fitted[1:]):
        distances.append(distances[-1] + math.dist(start, end))
    return fitted, distances


def track_point(track, progress):
    """Return the point at progress 0..1, moving evenly by track distance."""
    if track is None:
        angle = 2 * math.pi * progress
        radius = (1 + 0.16 * math.cos(2 * angle + 0.6) +
                  0.09 * math.sin(3 * angle))
        return 637 + 100 * radius * math.cos(angle), 268 + 78 * radius * math.sin(angle)
    points, distances = track
    distance = (progress % 1.0) * distances[-1]
    index = min(max(bisect.bisect_right(distances, distance) - 1, 0),
                len(points) - 2)
    segment = distances[index + 1] - distances[index]
    fraction = (distance - distances[index]) / segment if segment else 0.0
    return (points[index][0] + fraction * (points[index + 1][0] - points[index][0]),
            points[index][1] + fraction * (points[index + 1][1] - points[index][1]))


def render(track, speed, battery, temp, progress, frame, clock_text,
           rotate, speed_max, map_box, supersample_factor):
    """Render one 8-bit grayscale dashboard PNG from the supplied telemetry."""
    image = Image.new("L", (W, H), 255)
    draw = ImageDraw.Draw(image)
    map_box = tuple(map_box)
    map_region = (map_box[0] - 16, map_box[1] - 12,
                  map_box[2] + 14, map_box[3] + 10)
    ss = int(supersample_factor)

    # ---- top bar -------------------------------------------------------
    draw.text((16, 24), "SHELL ECO-MARATHON", font=font(26), fill=0, anchor="lm")
    draw.text((400, 24), f"#{frame}", font=font(22), fill=0, anchor="mm")
    if frame % 2:
        draw.rectangle((610, 14, 630, 34), fill=0)
    else:
        draw.rectangle((610, 14, 630, 34), outline=0, width=3)
    draw.text((784, 24), clock_text, font=font(26), fill=0, anchor="rm")
    draw.line((16, 46, 784, 46), fill=0, width=3)

    # ---- speed bar: 24 rising segments ---------------------------------
    segments, gap = 24, 5
    segment_width = (768 - gap * (segments - 1)) / segments
    lit = round((speed or 0.0) / speed_max * segments)
    for index in range(segments):
        x0 = 16 + index * (segment_width + gap)
        height = 14 + 26 * index / (segments - 1)
        box = (x0, 100 - height, x0 + segment_width, 100)
        if index < lit:
            draw.rectangle(box, fill=0)
        else:
            draw.rectangle(box, outline=0, width=2)

    # ---- big speed number ----------------------------------------------
    draw.text((243, 262), f"{speed:.0f}" if speed is not None else "--",
              font=font(270), fill=0, anchor="mm")
    draw.text((243, 418), "KM/H", font=font(44), fill=0, anchor="mm")

    # ---- battery card (inverts when low) -------------------------------
    low = battery is not None and battery < 20
    ink = card(draw, (16, 466, 470, 584), low)
    draw.text((32, 488), "BATTERY  LOW" if low else "BATTERY",
              font=font(22), fill=ink, anchor="lm")
    draw.text((32, 538), f"{battery:.0f}%" if battery is not None else "--",
              font=font(76), fill=ink, anchor="lm")
    full = math.ceil(battery / 10) if battery is not None else 0
    for index in range(10):
        x0 = 225 + index * 23
        box = (x0, 508, x0 + 18, 558)
        if index < full:
            draw.rectangle(box, fill=ink)
        else:
            draw.rectangle(box, outline=ink, width=2)

    # ---- temperature card (inverts when hot) ---------------------------
    hot = temp is not None and temp > 45
    ink = card(draw, (490, 412, 784, 584), hot)
    draw.text((506, 434), "TEMP  HIGH" if hot else "TEMP",
              font=font(22), fill=ink, anchor="lm")
    draw.text((506, 488), f"{temp:.0f}\u00b0C" if temp is not None else "--",
              font=font(84), fill=ink, anchor="lm")
    fraction = max(0.0, min(1.0, (temp - 20) / 40)) if temp is not None else 0.0
    draw.rectangle((506, 538, 768, 562), outline=ink, width=3)
    draw.rectangle((509, 541, 509 + int(256 * fraction), 559), fill=ink)

    # ---- map card: track and moving car -----------------------------
    draw.rounded_rectangle((490, 112, 784, 400), radius=14, outline=0, width=4)
    draw.text((506, 134), "TRACK", font=font(22), fill=0, anchor="lm")
    region_x0, region_y0, region_x1, region_y1 = map_region
    layer = Image.new("L", ((region_x1 - region_x0) * ss,
                             (region_y1 - region_y0) * ss), 255)
    layer_draw = ImageDraw.Draw(layer)

    def layer_point(point):
        return ((point[0] - region_x0) * ss, (point[1] - region_y0) * ss)

    points = track[0] if track else [track_point(None, index / 120)
                                     for index in range(121)]
    line_points = [layer_point(point) for point in points]
    road_width = 8 * ss
    layer_draw.line(line_points, fill=0, width=road_width, joint="curve")
    for point in (line_points[0], line_points[-1]):
        layer_draw.ellipse((point[0] - road_width / 2, point[1] - road_width / 2,
                            point[0] + road_width / 2, point[1] + road_width / 2), fill=0)
    start_x, start_y = layer_point(track_point(track, 0))
    next_x, next_y = layer_point(track_point(track, 0.004))
    norm = math.hypot(next_x - start_x, next_y - start_y) or 1.0
    normal_x, normal_y = -(next_y - start_y) / norm, (next_x - start_x) / norm
    layer_draw.line((start_x - 9 * ss * normal_x, start_y - 9 * ss * normal_y,
                     start_x + 9 * ss * normal_x, start_y + 9 * ss * normal_y),
                    fill=0, width=4 * ss)
    if progress is not None:
        car_x, car_y = layer_point(track_point(track, progress))
        layer_draw.ellipse((car_x - 14 * ss, car_y - 14 * ss,
                            car_x + 14 * ss, car_y + 14 * ss), fill=255)
        layer_draw.ellipse((car_x - 9 * ss, car_y - 9 * ss,
                            car_x + 9 * ss, car_y + 9 * ss), fill=0)
    image.paste(layer.resize((region_x1 - region_x0, region_y1 - region_y0),
                             Image.Resampling.LANCZOS), (region_x0, region_y0))

    output = image.rotate(rotate, expand=True) if rotate else image
    buffer = io.BytesIO()
    output.save(buffer, "PNG", compress_level=1)
    return buffer.getvalue()
