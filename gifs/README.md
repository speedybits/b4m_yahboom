# ROSIE GIF Animations

This directory contains animated GIF files that are displayed on the ROSIE web status interface.

## Required GIF Files

Place the following animated GIF files in this directory:

### 1. `waiting.gif`
**When displayed**: ROSIE is in LISTENING state, waiting for voice input
- No voice activity detected
- Listening for the wake word "Rosie"
- Suggested animation: Gentle breathing, blinking eyes, idle animation

### 2. `thinking.gif`
**When displayed**: ROSIE is in RESPONDING state, processing your request
- Received wake word and query
- Sending prompt to Ollama LLM
- Waiting for LLM response
- During "Let me think" (when summarizing conversation)
- Suggested animation: Thinking pose, gears turning, processing indicator

### 3. `speaking.gif`
**When displayed**: ROSIE is in SPEAKING state, delivering a response
- Text-to-speech is active
- Piper is generating and playing audio
- Suggested animation: Talking, moving mouth, sound waves

## GIF Requirements

- **Format**: Animated GIF (.gif extension)
- **Size**: Recommended 400x400 to 800x800 pixels
- **File size**: Keep under 5MB for fast loading
- **Loop**: Set to loop infinitely
- **Frame rate**: 10-30 fps works well

## Testing

After adding your GIFs:

1. Start the web server:
   ```bash
   python3 rosie_web_status.py
   ```

2. Open your browser to:
   ```
   http://localhost:5000
   ```

3. Start ROSIE:
   ```bash
   ./rosie_launch.sh
   ```

4. The web page will automatically update to show the appropriate GIF based on ROSIE's current state.

## Placeholder GIFs

If you don't have custom GIFs yet, you can:
- Create simple placeholder GIFs using any GIF maker tool
- Use colored squares or text labels as placeholders
- Download free animated GIFs from sites like Giphy (ensure licensing allows use)

The web interface will still function without GIFs, but images won't display until you add them.
