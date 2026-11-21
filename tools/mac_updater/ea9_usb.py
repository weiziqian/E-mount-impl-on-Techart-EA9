"""Mac USB HID transport and the EA9 updater's connection state."""

import hid

from ea9flash import Adapter, DEFAULT_PID, DEFAULT_VID
from firmware import REPORT_SIZE, check_size, transfer_size


class MacAdapter(Adapter):
    """Use the shared CHECK and START encodings over HIDAPI."""

    def __init__(self, path):
        self.timeout = 5.0
        self.device = hid.device()
        try:
            self.device.open_path(path)
        except OSError:
            self.device.close()
            raise

    def close(self):
        self.device.close()

    def send(self, payload):
        if len(payload) > REPORT_SIZE:
            raise ValueError("HID payload is too large.")
        # HIDAPI requires the zero report ID on writes. Input is unnumbered.
        report = b"\0" + payload.ljust(REPORT_SIZE, b"\0")
        if self.device.write(report) != len(report):
            raise OSError("Short USB HID write.")

    def recv(self, timeout=None):
        seconds = self.timeout if timeout is None else timeout
        report = bytes(self.device.read(REPORT_SIZE, max(1, int(seconds * 1000))))
        if not report:
            return None
        if len(report) != REPORT_SIZE:
            raise OSError(f"USB HID returned a short report ({len(report)} bytes).")
        return report


def connected_paths():
    return {item["path"] for item in hid.enumerate(DEFAULT_VID, DEFAULT_PID)}


def version_text(version):
    return ".".join(map(str, version))


class Connection:
    """Use from one worker thread, including status polls and updates."""

    def __init__(self):
        self.path = None
        self.ready = False
        self.status = ""

    def poll(self):
        paths = connected_paths()
        if self.path not in paths:
            self.path = None
            self.ready = False
        if not paths:
            return None, "Connect the LM-EA9 by USB with the camera and lens removed."
        if len(paths) != 1:
            return None, "Connect just one LM-EA9."
        path = next(iter(paths))
        if path != self.path:
            self.path = path
            try:
                with MacAdapter(path) as adapter:
                    version = adapter.check()
            except OSError as error:
                self.status = f"Adapter did not respond. Reconnect it to retry. ({error})"
            else:
                detail = ("no version reported (0.0.0)" if version == (0, 0, 0)
                          else version_text(version))
                self.status = f"Connected · Bootloader record {detail}"
                self.ready = True
        return self.path if self.ready else None, self.status

    def flash(self, expected_path, image, version, progress):
        check_size(image)
        if (not self.ready or self.path != expected_path
                or connected_paths() != {expected_path}):
            raise OSError("The connection changed. Check which adapter is connected and try again.")
        self.ready = False
        self.status = "Reconnect the adapter before trying again."
        total = transfer_size(len(image))
        with MacAdapter(expected_path) as adapter:
            adapter.check()
            adapter.start(version)
            for offset in range(0, total, REPORT_SIZE):
                try:
                    adapter.send(image[offset:offset + REPORT_SIZE])
                    if adapter.recv() is None:
                        raise TimeoutError("No acknowledgement from the adapter.")
                except OSError as error:
                    raise OSError(f"Transfer stopped after {offset} of {total} acknowledged bytes. "
                                  f"{error}") from error
                progress(offset + REPORT_SIZE, total)
        self.status = "Transfer complete. Unplug the adapter."
