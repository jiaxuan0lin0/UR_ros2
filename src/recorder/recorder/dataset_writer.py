"""Dataset writer for images and robot state records."""

import json
import queue
import shutil
import threading
import time
from pathlib import Path

import cv2

from recorder.config import (
    DATASET_ROOT,
    EPISODE_PREFIX,
    IMAGE_DIR_PREFIX,
    STATE_FILE_NAME,
)


class DatasetWriter:
    def __init__(self, dataset_root=DATASET_ROOT, max_queue_size: int = 256):
        timestamp = int(time.time() * 1000)
        self.episode_dir = Path(dataset_root) / f"{EPISODE_PREFIX}{timestamp}"
        self.episode_dir.mkdir(parents=True, exist_ok=True)

        self.records = []
        self.camera_dirs = {}
        self.camera_index = {}
        self.sample_queue = queue.Queue(maxsize=max_queue_size)
        self._sentinel = object()
        self._closed = False
        self._saved = False
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()

    def _get_camera_dir(self, serial: str) -> Path:
        if serial not in self.camera_index:
            self.camera_index[serial] = len(self.camera_index)

        if serial not in self.camera_dirs:
            cam_idx = self.camera_index[serial]
            cam_dir = self.episode_dir / f"{IMAGE_DIR_PREFIX}{cam_idx}"
            cam_dir.mkdir(parents=True, exist_ok=True)
            self.camera_dirs[serial] = cam_dir

        return self.camera_dirs[serial]

    def _write_sample(self, frame_idx: int, frames_dict: dict, record: dict):
        for serial, image in frames_dict.items():
            if image is None:
                continue

            image_path = self._get_camera_dir(serial) / f"{frame_idx:05d}.jpg"
            cv2.imwrite(str(image_path), image)

        self.records.append(record)

    def _worker_loop(self):
        while True:
            item = self.sample_queue.get()
            try:
                if item is self._sentinel:
                    break

                frame_idx, frames_dict, record = item
                self._write_sample(frame_idx, frames_dict, record)
            finally:
                self.sample_queue.task_done()

    def enqueue_sample(self, frame_idx: int, frames_dict: dict, record: dict):
        if self._closed:
            raise RuntimeError("DatasetWriter is already closed")

        frames_copy = {
            serial: None if image is None else image.copy()
            for serial, image in frames_dict.items()
        }
        self.sample_queue.put((frame_idx, frames_copy, dict(record)))

    def wait_until_empty(self):
        self.sample_queue.join()

    def _write_state_file(self):
        self.episode_dir.mkdir(parents=True, exist_ok=True)
        state_file = self.episode_dir / STATE_FILE_NAME
        with open(state_file, "w") as f:
            json.dump(self.records, f, indent=4)

    def close(self, save: bool = True) -> bool:
        if self._closed:
            return self._saved

        self._closed = True
        self.sample_queue.put(self._sentinel)
        self._worker.join()

        if save and self.records:
            self._write_state_file()
            self._saved = True
        else:
            shutil.rmtree(self.episode_dir, ignore_errors=True)
            self._saved = False
        return self._saved

    @property
    def closed(self) -> bool:
        return self._closed

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()
