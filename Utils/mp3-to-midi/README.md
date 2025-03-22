## Setup
Create Python virtual environment
```
python3 -m venv .venv
```

Activate the virtual environment and install required libraries
```
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage
```
python mp3_to_midi.py input.mp3 --output_file output.mid --sensitivity 0.5 --velocity 100
```

# Understanding Sensitivity and Velocity Parameters in MP3 to MIDI Conversion

The sensitivity and velocity flags in the MP3 to MIDI conversion script control different aspects of the output MIDI file, 
affecting how notes are detected and how they sound when played back. Let me explain each parameter and provide examples for 
different scenarios.

## Sensitivity Parameter (--sensitivity)

The sensitivity parameter (range 0.0-1.0) controls the threshold for note detection:

- **Higher values (closer to 1.0)**: More selective, only detecting notes with stronger signals
- **Lower values (closer to 0.0)**: More inclusive, detecting even weaker signals as notes

### Examples for Different Sensitivity Settings:

#### 1. High Sensitivity (0.8) - Sparse Notes
```
python mp3_to_midi.py piano_recording.mp3 --sensitivity 0.8
```
**Result**: Only the most prominent notes will be detected. This works well for:
- Clean solo instrument recordings
- When you want to extract only the melody line
- Avoiding "ghost notes" or background noise
- Simplifying a complex piece

#### 2. Medium Sensitivity (0.5) - Balanced Detection
```
python mp3_to_midi.py guitar_song.mp3 --sensitivity 0.5
```
**Result**: A balance between including important notes while filtering some noise. Good for:
- Most music with moderate complexity
- Semi-clean recordings
- Getting both melody and some harmonies

#### 3. Low Sensitivity (0.2) - Detailed Capture
```
python mp3_to_midi.py orchestral_piece.mp3 --sensitivity 0.2
```
**Result**: Captures more subtle notes and harmonics. Useful for:
- Complex pieces with many instruments
- When you want to capture background harmonies
- Creating a richer MIDI representation
- When the source audio is very clean

## Velocity Parameter (--velocity)

Velocity (range 0-127) determines how "hard" each note is struck - affecting the volume and sometimes timbre when played back:

- **Higher values (80-127)**: Louder, more forceful notes
- **Lower values (1-40)**: Softer, more gentle notes
- **Mid values (40-80)**: Moderate volume

### Examples for Different Velocity Settings:

#### 1. High Velocity (110) - Bold, Loud Sound
```
python mp3_to_midi.py rock_song.mp3 --velocity 110
```
**Result**: Creates a MIDI with strong, prominent notes. Appropriate for:
- Rock, metal, or energetic music
- When you want the melody to stand out
- Compensating for quiet source recordings
- Creating dramatic effect

#### 2. Medium Velocity (80) - Standard Dynamic
```
python mp3_to_midi.py pop_song.mp3 --velocity 80
```
**Result**: Creates a balanced sound similar to normal playing. Good for:
- Most popular music genres
- General purpose conversions
- When you're unsure what to choose

#### 3. Low Velocity (40) - Soft, Gentle Sound
```
python mp3_to_midi.py ambient_music.mp3 --velocity 40
```
**Result**: Creates a softer, more delicate sound. Ideal for:
- Ambient or relaxing music
- Creating a background track
- Classical pieces with subtle dynamics
- Creating a dreamy or distant sound effect

## Combining Both Parameters for Specific Results

You can combine these parameters to achieve different effects:

### Example Combinations:

#### 1. Clean Solo Instrument Transcription
```
python mp3_to_midi.py piano_solo.mp3 --sensitivity 0.7 --velocity 90
```
**Effect**: Captures primarily the intended notes with natural dynamics, filtering out background noise while preserving the 
performance intensity.

#### 2. Extracting Only Main Melody from a Song
```
python mp3_to_midi.py vocal_track.mp3 --sensitivity 0.85 --velocity 100
```
**Effect**: Focuses on only the strongest signal (likely the vocal melody) and makes it prominent.

#### 3. Rich Orchestral Conversion
```
python mp3_to_midi.py symphony.mp3 --sensitivity 0.3 --velocity 75
```
**Effect**: Captures more of the harmonies and background parts while keeping a balanced dynamic range.

#### 4. Creating a "Ghostly" Version
```
python mp3_to_midi.py any_song.mp3 --sensitivity 0.4 --velocity 30
```
**Effect**: Captures a fair amount of detail but makes everything sound soft and distant.

## Tips for Best Results

1. **Start with defaults** (sensitivity 0.5, velocity 100) and adjust based on results
2. **For cleaner MIDI files**: Increase sensitivity to reduce unwanted notes
3. **For fuller sound**: Decrease sensitivity to capture more notes
4. **Match the velocity to the genre**: higher for energetic music, lower for calm pieces
5. **Experiment**: Try different combinations to find what works best for your specific audio
