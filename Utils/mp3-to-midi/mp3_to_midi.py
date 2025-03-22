import os
import argparse
import numpy as np
from midiutil import MIDIFile
from pydub import AudioSegment
import librosa
import librosa.display

def mp3_to_midi(input_file, output_file, sensitivity=0.5, velocity=100):
    """
    Convert MP3 audio to MIDI.
    
    Args:
        input_file (str): Path to input MP3 file
        output_file (str): Path to output MIDI file
        sensitivity (float): Threshold for note detection (0.0-1.0)
        velocity (int): MIDI velocity (0-127)
    """
    print(f"Converting {input_file} to MIDI...")
    
    # Load the audio file
    y, sr = librosa.load(input_file, sr=None)
    
    # Extract pitch and onset information
    onset_frames = librosa.onset.onset_detect(y=y, sr=sr)
    onset_times = librosa.frames_to_time(onset_frames, sr=sr)
    
    # Extract pitches using harmonic-percussive source separation
    y_harmonic = librosa.effects.harmonic(y)
    pitches, magnitudes = librosa.core.piptrack(y=y_harmonic, sr=sr)
    
    # Create a MIDI file with one track
    midi = MIDIFile(1)
    track = 0
    time = 0
    midi.addTrackName(track, time, "MIDI Track")
    midi.addTempo(track, time, 120)  # Default tempo
    
    last_onset_time = 0
    
    # Process each onset
    for i, onset_time in enumerate(onset_times):
        # Find the pitches at the onset time
        frame_idx = librosa.time_to_frames(onset_time, sr=sr)
        if frame_idx >= pitches.shape[1]:
            frame_idx = pitches.shape[1] - 1
            
        # Find the highest magnitude pitch at this frame
        pitch_idx = np.argmax(magnitudes[:, frame_idx])
        if magnitudes[pitch_idx, frame_idx] > sensitivity:
            pitch = pitches[pitch_idx, frame_idx]
            # Convert frequency to MIDI note number
            midi_note = int(round(librosa.hz_to_midi(pitch)))
            
            # Calculate duration until next onset
            if i < len(onset_times) - 1:
                duration = onset_times[i + 1] - onset_time
            else:
                duration = 0.5  # Default duration for the last note
                
            # Add the note to the MIDI file (convert time to beats assuming 120 BPM)
            midi_time = onset_time * 2  # Convert seconds to beats at 120 BPM
            midi_duration = duration * 2  # Same conversion for duration
            
            # Ensure the note is within MIDI range (0-127)
            if 0 <= midi_note <= 127:
                midi.addNote(track, 0, midi_note, midi_time, midi_duration, velocity)
                
            last_onset_time = onset_time
    
    # Write the MIDI file
    with open(output_file, "wb") as output_file:
        midi.writeFile(output_file)
    
    print(f"MIDI file saved as {output_file}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Convert MP3 to MIDI')
    parser.add_argument('input_file', help='Input MP3 file path')
    parser.add_argument('--output_file', help='Output MIDI file path')
    parser.add_argument('--sensitivity', type=float, default=0.5, 
                        help='Sensitivity threshold (0.0-1.0)')
    parser.add_argument('--velocity', type=int, default=100, 
                        help='MIDI note velocity (0-127)')
    
    args = parser.parse_args()
    
    # If output file not specified, use input filename with .mid extension
    if not args.output_file:
        args.output_file = os.path.splitext(args.input_file)[0] + '.mid'
    
    mp3_to_midi(args.input_file, args.output_file, args.sensitivity, args.velocity)
