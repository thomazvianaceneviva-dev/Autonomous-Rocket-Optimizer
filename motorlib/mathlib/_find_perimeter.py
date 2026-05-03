import numpy as np
try:
    from ._find_perimeter_cy import _get_perimeter
except ImportError:
    _get_perimeter = None
from collections import deque
import numpy as np
from skimage.measure import find_contours as sk_find_contours

def find_perimeter(image, level,
                  *,
                  including_contours=False,
                  fully_connected='low'):

    if image.shape[0] < 2 or image.shape[1] < 2:
        raise ValueError("Input array must be at least 2x2.")

    if image.ndim != 2:
        raise ValueError("Only 2D arrays are supported.")

    # Use fast Cython version if available
    if _get_perimeter is not None:
        perimeter, segments = _get_perimeter(
            image,
            float(level),
            fully_connected == 'high',
            including_contours
        )

        contours = []
        if including_contours:
            contours = _assemble_contours(segments)

        return perimeter, contours

    # fallback Python puro
    contours = sk_find_contours(image, level)

    perimeter = 0.0

    for contour in contours:
        diffs = np.diff(contour, axis=0)
        seg_lengths = np.sqrt((diffs ** 2).sum(axis=1))
        perimeter += seg_lengths.sum()

    if including_contours:
        return perimeter, contours

    return perimeter, []


def _assemble_contours(segments):
    current_index = 0
    contours = {}
    starts = {}
    ends = {}
    for from_point, to_point in segments:
        # Ignore degenerate segments.
        # This happens when (and only when) one vertex of the square is
        # exactly the contour level, and the rest are above or below.
        # This degenerate vertex will be picked up later by neighboring
        # squares.
        if from_point == to_point:
            continue

        tail, tail_num = starts.pop(to_point, (None, None))
        head, head_num = ends.pop(from_point, (None, None))

        if tail is not None and head is not None:
            # We need to connect these two contours.
            if tail is head:
                # We need to closed a contour: add the end point
                head.append(to_point)
            else:  # tail is not head
                # We need to join two distinct contours.
                # We want to keep the first contour segment created, so that
                # the final contours are ordered left->right, top->bottom.
                if tail_num > head_num:
                    # tail was created second. Append tail to head.
                    head.extend(tail)
                    # Remove tail from the detected contours
                    contours.pop(tail_num, None)
                    # Update starts and ends
                    starts[head[0]] = (head, head_num)
                    ends[head[-1]] = (head, head_num)
                else:  # tail_num <= head_num
                    # head was created second. Prepend head to tail.
                    tail.extendleft(reversed(head))
                    # Remove head from the detected contours
                    starts.pop(head[0], None)  # head[0] can be == to_point!
                    contours.pop(head_num, None)
                    # Update starts and ends
                    starts[tail[0]] = (tail, tail_num)
                    ends[tail[-1]] = (tail, tail_num)
        elif tail is None and head is None:
            # We need to add a new contour
            new_contour = deque((from_point, to_point))
            contours[current_index] = new_contour
            starts[from_point] = (new_contour, current_index)
            ends[to_point] = (new_contour, current_index)
            current_index += 1
        elif head is None:  # tail is not None
            # tail first element is to_point: the new segment should be
            # prepended.
            tail.appendleft(from_point)
            # Update starts
            starts[from_point] = (tail, tail_num)
        else:  # tail is None and head is not None:
            # head last element is from_point: the new segment should be
            # appended
            head.append(to_point)
            # Update ends
            ends[to_point] = (head, head_num)

    return [np.array(contour) for _, contour in sorted(contours.items())]
