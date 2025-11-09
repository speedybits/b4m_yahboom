# ROSIE Image Animations

This directory contains numbered image files that are cycled randomly to create animated effects on the ROSIE web status interface.

## Required Image Files

Place numbered image files in this directory for each of ROSIE's three states:

### 1. Waiting Images (`waiting1.png`, `waiting2.png`, etc.)
**When displayed**: ROSIE is in LISTENING state, waiting for voice input
- No voice activity detected
- Listening for the wake word "Rosie"
- Suggested content: Gentle breathing, blinking eyes, idle poses, calm expressions

**Examples:**
- `waiting1.png` - Eyes open
- `waiting2.png` - Eyes half-closed
- `waiting3.png` - Eyes blinking

### 2. Thinking Images (`thinking1.png`, `thinking2.png`, etc.)
**When displayed**: ROSIE is in RESPONDING state, processing your request
- Received wake word and query
- Sending prompt to Ollama LLM
- Waiting for LLM response
- During "Let me think" (when summarizing conversation)
- Suggested content: Thinking poses, gears turning, processing indicators, concentrated expressions

**Examples:**
- `thinking1.png` - Looking up thoughtfully
- `thinking2.png` - Hand on chin
- `thinking3.png` - Gears spinning

### 3. Speaking Images (`speaking1.png`, `speaking2.png`, etc.)
**When displayed**: ROSIE is in SPEAKING state, delivering a response
- Text-to-speech is active
- Piper is generating and playing audio
- Suggested content: Talking, mouth movements, sound waves, expressive gestures

**Examples:**
- `speaking1.png` - Mouth open
- `speaking2.png` - Mouth forming words
- `speaking3.png` - Sound waves effect

## Image Requirements

### File Naming
- **Pattern**: `{state}{number}.{ext}`
- **States**: `waiting`, `thinking`, `speaking` (lowercase)
- **Numbers**: Start from 1, consecutive (1, 2, 3, 4, ...)
- **Extensions**: `.png`, `.jpg`, `.jpeg` (case-insensitive)

**Valid examples:**
- `waiting1.png`, `waiting2.png`, `waiting3.png`
- `thinking1.jpg`, `thinking2.jpg`
- `speaking1.PNG`, `speaking2.png`, `speaking3.jpeg`

**Invalid examples:**
- `Waiting1.png` (capital 'W' - won't match)
- `waiting_1.png` (underscore not allowed)
- `waiting.png` (no number)
- `talking1.png` (wrong state name)

### Format Specifications
- **Supported formats**: PNG (recommended), JPG/JPEG
- **Recommended size**: 400x400 to 800x800 pixels
- **File size**: Keep under 2MB per image for fast loading
- **Aspect ratio**: Square (1:1) works best
- **Background**: Transparent PNG recommended (or solid color for JPG)

### How Many Images?
- **Minimum**: 1 image per state (e.g., `waiting1.png`)
- **Recommended**: 3-5 images per state for good variety
- **Maximum**: No hard limit, but 10-20 images is practical
- **Different counts**: Each state can have a different number of images
  - Example: 3 waiting images, 2 thinking images, 4 speaking images

## How It Works

1. **Startup**: Web server scans this directory and finds all numbered images
2. **Grouping**: Images grouped by state (waiting, thinking, speaking)
3. **Sorting**: Images sorted numerically (waiting1, waiting2, waiting3...)
4. **Cycling**: Browser randomly selects images every 0.5-1 second
5. **No Repeats**: Won't show the same image twice in a row

**Example with 3 waiting images:**
```
Current: waiting2.png
Next options: [waiting1.png, waiting3.png]  (excludes waiting2.png)
Random pick: waiting3.png
Wait: Random 500-1000ms
Repeat...
```

## Testing

After adding your images:

1. Start the web server:
   ```bash
   python3 rosie_web_status.py
   ```

   You should see:
   ```
   [IMAGES] Found 3 images for 'waiting': ['waiting1.png', 'waiting2.png', 'waiting3.png']
   [IMAGES] Found 2 images for 'thinking': ['thinking1.png', 'thinking2.png']
   [IMAGES] Found 4 images for 'speaking': ['speaking1.jpg', 'speaking2.jpg', ...]
   ```

2. Open your browser to:
   ```
   http://localhost:5000
   ```

3. Start ROSIE:
   ```bash
   ./rosie_launch.sh
   ```

4. Watch the browser - images should cycle randomly based on ROSIE's state

## Creating Images

### Quick Start Options

1. **Use existing images/photos**
   - Find or create 3-5 images that represent each state
   - Resize to square aspect ratio (e.g., 600x600px)
   - Rename following the pattern: `waiting1.png`, `waiting2.png`, etc.

2. **AI-generated images**
   - Use tools like DALL-E, Midjourney, or Stable Diffusion
   - Generate variations of idle/thinking/talking poses
   - Download and rename appropriately

3. **Simple colored squares (testing)**
   - Create basic colored squares in any image editor
   - Label them: "WAITING 1", "WAITING 2", etc.
   - Use different colors for different states

4. **Animated GIF frames**
   - Extract frames from animated GIFs
   - Save each frame as separate numbered files
   - Example: Extract 5 frames from talking.gif → talking1.png through talking5.png

### Tools
- **Image editors**: GIMP (free), Photoshop, Canva
- **GIF frame extraction**: ezgif.com, GIMP
- **Resize**: ImageMagick, online tools
- **AI generation**: DALL-E, Midjourney, Stable Diffusion

## Troubleshooting

**No images displaying:**
- Check file names match pattern exactly: `waiting1.png` not `Waiting1.png`
- Check browser console (F12) for errors
- Verify images are in the `gifs/` directory
- Check web server startup output for "Found X images" messages

**Images not cycling:**
- Open browser console (F12) and look for "Cycling to image" messages
- Check that you have more than 1 image for the state
- Verify ROSIE is actually changing states

**Slow loading:**
- Reduce image file sizes (compress or resize)
- Use PNG format with transparency instead of large JPGs
- Aim for under 500KB per image

## Example Directory Structure

```
gifs/
├── .gitkeep
├── README.md (this file)
├── waiting1.png
├── waiting2.png
├── waiting3.png
├── thinking1.jpg
├── thinking2.jpg
├── speaking1.png
├── speaking2.png
├── speaking3.png
└── speaking4.png
```

This setup gives you:
- 3 waiting images
- 2 thinking images
- 4 speaking images

The web interface will cycle through each set randomly when ROSIE enters that state!
