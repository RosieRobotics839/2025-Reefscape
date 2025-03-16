## Student Tasks

1. Install required packages:
   ```
   pip install -r requirements.txt
   ```

2. Complete the TODO sections in the code:
   - Set the correct robot IP and serial port
   - Add the specific data points you want to read from NetworkTables
   - Format the data to match what your Arduino expects
   - Adjust timing as needed

3. Create a simple Arduino sketch that:
   - Reads the serial data
   - Parses the JSON
   - Uses the data (display on LCD, control LEDs, etc.)

4. Make the program run on startup:
   - Create a Windows batch file or shortcut
   - Add it to the Windows Startup folder

## Setting Up for Auto-Start

### Option 1
Create a file named `start_relay.bat` with:
```batch
@echo off
cd C:\path\to\your\project
python main.py
```

Place this in: `C:\Users\[username]\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Startup`

### Option 2
1. **Install PyInstaller**:
   ```
   pip install pyinstaller
   ```

2. **Create the executable**:
   ```
   pyinstaller --onefile --noconsole your_script.py
   ```
   - `--onefile`: Creates a single .exe file
   - `--noconsole`: Hides the console window (for GUI apps)
   - Add `--icon=your_icon.ico` for custom icons

3. **Configure startup**:
   - Create a shortcut to your .exe in the Windows Startup folder:
     `C:\Users\[Username]\AppData\Roaming\Microsoft\Windows\Start Menu\Programs\Startup`
   - OR use Windows Task Scheduler:
     - Open Task Scheduler → Create Basic Task
     - Set trigger to "At startup"
     - Action: "Start a program" → browse to your .exe



## Testing Tips

1. Test with the robot simulator first if available
2. Use print statements to debug data flow
3. Start with simple data before adding complexity

Good luck, you can do this!!


