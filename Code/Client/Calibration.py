# -*- coding: utf-8 -*-

from PyQt5 import QtCore, QtGui, QtWidgets

class Ui_calibration:
    """UI class for the calibration window, handling all UI element setup and configuration."""
    
    def __init__(self, calibration):
        self.calibration = calibration
        self.setupUi(calibration)
    
    def setupUi(self, calibration):
        """Initialize and configure all UI components."""
        self._setup_main_window(calibration)
        self._setup_radio_buttons(calibration)
        self._setup_input_fields(calibration)
        self._setup_buttons(calibration)
        self._setup_labels(calibration)
        self._setup_picture_label(calibration)
        self.retranslateUi(calibration)
        QtCore.QMetaObject.connectSlotsByName(calibration)
    
    def _setup_main_window(self, window):
        """Configure the main window properties."""
        window.setObjectName("calibration")
        window.resize(697, 353)
        window.setFont(QtGui.QFont("Arial"))
        window.setStyleSheet(self._get_stylesheet())
    
    def _get_stylesheet(self):
        """Return the stylesheet for the application."""
        return """
            QWidget { background: #484848; }
            QAbstractButton {
                border: none; border-radius: 0; padding: 5px;
                color: #DCDCDC;
                background: qlineargradient(
                    spread:pad, x1:0, y1:0, x2:0, y2:1,
                    stop:0 #858585, stop:1 #383838
                );
            }
            QAbstractButton:hover { color: #000; background: #008aff; }
            QAbstractButton:pressed {
                color: #DCDCDC; border-left: 4px solid #008aff;
                padding: 4px 4px 4px 2px; background: #444;
            }
            QLabel { color: #DCDCDC; }
            QLabel:focus { border: 1px solid #00BB9E; }
            QLineEdit {
                border: 1px solid #242424; border-radius: 3px; padding: 2px;
                background: none; selection-background-color: #484848;
                selection-color: #DCDCDC; lineedit-password-character: 9679;
            }
            QLineEdit:focus, QLineEdit:hover { border: 1px solid #242424; }
        """
    
    def _setup_radio_buttons(self, parent):
        """Initialize and configure radio buttons."""
        buttons = [
            ("one", 30, 25), ("two", 30, 55), ("three", 30, 85),
            ("four", 30, 115), ("five", 30, 145), ("six", 30, 175)
        ]
        for name, x, y in buttons:
            btn = self._create_radio_button(parent, x, y)
            setattr(self, f"radioButton_{name}", btn)
    
    def _create_radio_button(self, parent, x, y):
        """Helper to create a radio button with consistent styling."""
        btn = QtWidgets.QRadioButton(parent)
        btn.setGeometry(QtCore.QRect(x, y, 70, 20))
        btn.setFont(self._get_standard_font())
        btn.setStyleSheet("font: 10pt 'Arial';")
        return btn
    
    def _setup_input_fields(self, parent):
        """Initialize and configure all input fields."""
        # X, Y, Z fields for each radio button (1-6)
        for i in range(1, 7):
            # X coordinate
            x_field = self._create_input_field(parent, 135, 25 + (i-1)*30)
            setattr(self, f"{self._get_number_name(i)}_x", x_field)
            
            # Y coordinate
            y_field = self._create_input_field(parent, 210, 25 + (i-1)*30)
            setattr(self, f"{self._get_number_name(i)}_y", y_field)
            
            # Z coordinate
            z_field = self._create_input_field(parent, 285, 25 + (i-1)*30)
            setattr(self, f"{self._get_number_name(i)}_z", z_field)
    
    def _create_input_field(self, parent, x, y):
        """Helper to create an input field with consistent styling."""
        field = QtWidgets.QLineEdit(parent)
        field.setGeometry(QtCore.QRect(x, y, 45, 20))
        field.setFont(self._get_standard_font())
        field.setStyleSheet("font: 10pt 'Arial';")
        field.setAlignment(QtCore.Qt.AlignCenter)
        field.setText("0")  # Default value
        if "y" in field.objectName() and not field.objectName().startswith("six"):
            field.setText("72")  # Set default Y value to 72
        return field
    
    def _setup_buttons(self, parent):
        """Initialize and configure action buttons."""
        buttons = [
            ("X1", 230, 235, 90, 30), ("Z1", 230, 275, 90, 30),
            ("Y2", 130, 225, 90, 30), ("Z2", 30, 275, 90, 30),
            ("X2", 30, 235, 90, 30), ("Y1", 130, 305, 90, 30),
            ("Save", 130, 265, 90, 30)
        ]
        for name, x, y, w, h in buttons:
            btn = self._create_button(parent, x, y, w, h)
            setattr(self, f"Button_{name}", btn)
    
    def _create_button(self, parent, x, y, w, h):
        """Helper to create a button with consistent styling."""
        btn = QtWidgets.QPushButton(parent)
        btn.setGeometry(QtCore.QRect(x, y, w, h))
        btn.setFont(self._get_standard_font())
        btn.setStyleSheet("font: 10pt 'Arial';")
        return btn
    
    def _setup_labels(self, parent):
        """Initialize and configure all labels."""
        # X, Y, Z labels for each row
        for i in range(1, 7):
            y_pos = 30 + (i-1)*30
            # X label
            x_label = self._create_label(parent, "X:", 110, y_pos)
            setattr(self, f"label{'' if i == 1 else f'_{i*3-3}' if i < 6 else '_13' if i == 6 else ''}", x_label)
            
            # Y label
            y_label = self._create_label(parent, "Y:", 190, y_pos)
            setattr(self, f"label{'_2' if i == 1 else f'_{i*3-2}' if i < 6 else '_15' if i == 6 else ''}", y_label)
            
            # Z label
            z_label = self._create_label(parent, "Z:", 265, y_pos)
            setattr(self, f"label{'_3' if i == 1 else f'_{i*3-1}' if i < 6 else '_18' if i == 6 else ''}", z_label)
    
    def _create_label(self, parent, text, x, y):
        """Helper to create a label with consistent styling."""
        label = QtWidgets.QLabel(parent)
        label.setGeometry(QtCore.QRect(x, y, 15, 12))
        label.setFont(self._get_standard_font())
        label.setStyleSheet("font: 10pt 'Arial';")
        label.setText(text)
        return label
    
    def _setup_picture_label(self, parent):
        """Initialize the picture display label."""
        self.label_picture = QtWidgets.QLabel(parent)
        self.label_picture.setGeometry(QtCore.QRect(355, 20, 320, 320))
        self.label_picture.setFont(self._get_standard_font())
        self.label_picture.setStyleSheet("font: 10pt 'Arial';")
        self.label_picture.setText("")
    
    @staticmethod
    def _get_standard_font():
        """Return a standard font configuration."""
        font = QtGui.QFont()
        font.setFamily("Arial")
        font.setPointSize(9)
        font.setBold(False)
        font.setItalic(False)
        font.setWeight(50)
        return font
    
    @staticmethod
    def _get_number_name(num):
        """Convert number to its word representation for attribute names."""
        numbers = {1: 'one', 2: 'two', 3: 'three', 4: 'four', 5: 'five', 6: 'six'}
        return numbers.get(num, '')
    
    def retranslateUi(self, calibration):
        """Set up translations and text content for all UI elements."""
        _translate = QtCore.QCoreApplication.translate
        calibration.setWindowTitle(_translate("calibration", "Calibration"))
        
        # Set radio button texts
        for i, name in enumerate(['One', 'Two', 'Three', 'Four', 'Five', 'Six'], 1):
            getattr(self, f"radioButton_{name.lower()}").setText(_translate("calibration", name))
        
        # Set button texts
        button_texts = {
            'X1': "X+", 'X2': "X-", 'Y1': "Y+", 'Y2': "Y-",
            'Z1': "Z+", 'Z2': "Z-", 'Save': "Save"
        }
        for btn_name, text in button_texts.items():
            getattr(self, f"Button_{btn_name}").setText(_translate("calibration", text))
        
        # Set coordinate labels
        for i in range(1, 7):
            for coord in ['x', 'y', 'z']:
                field = getattr(self, f"{self._get_number_name(i)}_{coord}")
                field.setText(_translate("calibration", "72" if coord == 'y' and i != 6 else "0"))