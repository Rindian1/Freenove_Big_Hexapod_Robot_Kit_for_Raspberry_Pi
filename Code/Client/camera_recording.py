# -*- coding: utf-8 -*-
from __future__ import annotations

import os
from datetime import datetime
from typing import Optional

from PyQt5 import QtCore, QtGui
from PyQt5.QtGui import QPixmap, QImage


class CameraRecorder:
    """
    Save the current camera frame displayed in a QLabel as an image.
    If no frame is available, save a blank image with the label's size.
    """

    def __init__(
        self,
        output_dir: str = "Captures",
        video_label: Optional[QtGui.QWindow] = None,
        fallback_size: Optional[QtCore.QSize] = None,
        blank_color: QtGui.QColor = QtGui.QColor("black"),
    ) -> None:
        self.output_dir = output_dir
        self.video_label = video_label
        self.fallback_size = fallback_size or QtCore.QSize(600, 450)
        self.blank_color = blank_color

        # Ensure output directory exists
        try:
            os.makedirs(self.output_dir, exist_ok=True)
        except Exception:
            pass

    def _timestamp_name(self) -> str:
        return datetime.now().strftime("Capture_%Y%m%d_%H%M%S.png")

    def _blank_pixmap(self) -> QPixmap:
        size = self.fallback_size
        if self.video_label is not None:
            try:
                # Use current label size if available
                size = self.video_label.size()
            except Exception:
                pass
        img = QImage(size.width(), size.height(), QImage.Format_RGB32)
        img.fill(self.blank_color)
        return QPixmap.fromImage(img)

    def capture(self, pixmap: Optional[QPixmap] = None) -> str:
        """
        Save the provided pixmap. If it's None or null, create and save a blank image instead.
        Returns the absolute file path of the saved image.
        """
        if pixmap is None or pixmap.isNull():
            pixmap = self._blank_pixmap()

        filename = self._timestamp_name()
        full_path = os.path.join(self.output_dir, filename)
        # Prefer PNG for lossless snapshot
        pixmap.save(full_path, "PNG")
        return os.path.abspath(full_path)
