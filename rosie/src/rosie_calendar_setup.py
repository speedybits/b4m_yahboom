#!/usr/bin/env python3
"""
ROSIE Calendar Setup Wizard
Interactive setup for Google Calendar integration with ROSIE

This script:
1. Guides you through Google Calendar API authentication
2. Lists all available calendars (including shared ones)
3. Lets you select which calendars to sync with ROSIE
4. Creates calendar_config.json with your selections
"""

import os
import json
from pathlib import Path
from google.auth.transport.requests import Request
from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from googleapiclient.discovery import build
from googleapiclient.errors import HttpError

# Scopes for read/write access to calendars
SCOPES = ['https://www.googleapis.com/auth/calendar']

# File paths (relative to rosie root directory - parent of src/)
ROSIE_DIR = Path(__file__).parent.parent
CONFIG_FILE = ROSIE_DIR / 'data' / 'calendar_config.json'


def get_client_config():
    """
    Build OAuth client configuration from environment variables

    Required environment variables in ~/.bashrc:
        GOOGLE_CALENDAR_CLIENT_ID
        GOOGLE_CALENDAR_CLIENT_SECRET

    Returns:
        Dictionary with OAuth client configuration
    """
    client_id = os.getenv('GOOGLE_CALENDAR_CLIENT_ID')
    client_secret = os.getenv('GOOGLE_CALENDAR_CLIENT_SECRET')

    if not client_id or not client_secret:
        print("\n[ERROR] Google Calendar credentials not found in environment variables!")
        print("\nPlease add these to your ~/.bashrc:")
        print("  export GOOGLE_CALENDAR_CLIENT_ID='your-client-id-here'")
        print("  export GOOGLE_CALENDAR_CLIENT_SECRET='your-client-secret-here'")
        print("\nTo get these credentials:")
        print("1. Go to https://console.cloud.google.com/")
        print("2. Create a new project (or select existing)")
        print("3. Enable Google Calendar API")
        print("4. Create OAuth 2.0 Desktop credentials")
        print("5. Copy the Client ID and Client Secret")
        print("6. Add them to ~/.bashrc as shown above")
        print("7. Run: source ~/.bashrc")
        return None

    # Build OAuth client config dictionary
    # This matches the structure of credentials.json for "installed" app type
    return {
        "installed": {
            "client_id": client_id,
            "client_secret": client_secret,
            "auth_uri": "https://accounts.google.com/o/oauth2/auth",
            "token_uri": "https://oauth2.googleapis.com/token",
            "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
            "redirect_uris": ["http://localhost"]
        }
    }


def get_token_data():
    """
    Get stored token data from environment variable

    Token is stored as JSON string in GOOGLE_CALENDAR_TOKEN environment variable.
    This contains the access token, refresh token, and expiry information.

    Returns:
        Dictionary with token data, or None if not found
    """
    token_json = os.getenv('GOOGLE_CALENDAR_TOKEN')
    if token_json:
        try:
            return json.loads(token_json)
        except json.JSONDecodeError:
            print("[AUTH] Warning: Invalid token data in GOOGLE_CALENDAR_TOKEN")
            return None
    return None


def save_token_data(creds):
    """
    Save token data to environment variable (for this session)

    Note: User must manually copy the token to their ~/.bashrc for persistence.

    Args:
        creds: Credentials object with token data
    """
    token_json = creds.to_json()

    # Set for current session
    os.environ['GOOGLE_CALENDAR_TOKEN'] = token_json

    # Print instructions for user to save permanently
    print("\n" + "=" * 80)
    print("TOKEN GENERATED - SAVE TO ~/.bashrc FOR PERSISTENCE")
    print("=" * 80)
    print("\nAdd this line to your ~/.bashrc:")
    print(f"\nexport GOOGLE_CALENDAR_TOKEN='{token_json}'")
    print("\nThen run: source ~/.bashrc")
    print("\nIMPORTANT: Keep this token secret - treat it like a password!")
    print("=" * 80)


def authenticate():
    """
    Authenticate with Google Calendar API using OAuth2 with environment variables

    Returns:
        Credentials object for API access
    """
    creds = None

    # Get OAuth client config from environment variables
    client_config = get_client_config()
    if not client_config:
        return None

    # Check if we have existing token from environment variable
    token_data = get_token_data()
    if token_data:
        print(f"[AUTH] Loading existing credentials from GOOGLE_CALENDAR_TOKEN")
        creds = Credentials.from_authorized_user_info(token_data, SCOPES)

    # If credentials are missing or invalid, run OAuth flow
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            print("[AUTH] Refreshing expired credentials...")
            creds.refresh(Request())
            save_token_data(creds)
        else:
            print("\n[AUTH] Starting OAuth2 authentication flow...")
            print("[AUTH] A browser window will open for you to authorize access")
            flow = InstalledAppFlow.from_client_config(client_config, SCOPES)
            creds = flow.run_local_server(port=0)
            save_token_data(creds)

    print("[AUTH] ✓ Authentication successful!")
    return creds


def list_calendars(service):
    """
    List all calendars accessible to the authenticated user

    Args:
        service: Google Calendar API service object

    Returns:
        List of calendar dictionaries with 'id', 'summary', 'primary' fields
    """
    try:
        print("\n[CALENDAR] Fetching your calendar list...")
        calendar_list = service.calendarList().list().execute()
        calendars = calendar_list.get('items', [])

        if not calendars:
            print("[CALENDAR] No calendars found")
            return []

        print(f"[CALENDAR] Found {len(calendars)} calendars\n")
        return calendars

    except HttpError as error:
        print(f"[ERROR] Failed to list calendars: {error}")
        return []


def display_calendars(calendars):
    """
    Display calendars in a numbered list for user selection

    Args:
        calendars: List of calendar dictionaries
    """
    print("=" * 80)
    print("AVAILABLE CALENDARS")
    print("=" * 80)

    for i, calendar in enumerate(calendars, 1):
        name = calendar.get('summary', 'Unnamed')
        cal_id = calendar.get('id', '')
        is_primary = calendar.get('primary', False)
        access_role = calendar.get('accessRole', 'unknown')

        primary_marker = " [PRIMARY]" if is_primary else ""

        print(f"\n{i}. {name}{primary_marker}")
        print(f"   ID: {cal_id}")
        print(f"   Access: {access_role}")

    print("\n" + "=" * 80)


def select_calendars(calendars):
    """
    Interactive calendar selection

    Args:
        calendars: List of available calendars

    Returns:
        List of selected calendar dictionaries
    """
    display_calendars(calendars)

    print("\nWhich calendars would you like to sync with ROSIE?")
    print("Enter calendar numbers separated by commas (e.g., 1,3,5)")
    print("Or press Enter to select all calendars")

    while True:
        selection = input("\nYour selection: ").strip()

        # Empty selection = all calendars
        if not selection:
            print(f"[SELECTION] Selected all {len(calendars)} calendars")
            return calendars

        # Parse comma-separated numbers
        try:
            indices = [int(x.strip()) for x in selection.split(',')]

            # Validate indices
            if all(1 <= i <= len(calendars) for i in indices):
                selected = [calendars[i-1] for i in indices]
                print(f"\n[SELECTION] Selected {len(selected)} calendar(s):")
                for cal in selected:
                    print(f"  - {cal.get('summary', 'Unnamed')}")
                return selected
            else:
                print(f"[ERROR] Please enter numbers between 1 and {len(calendars)}")

        except ValueError:
            print("[ERROR] Invalid input. Please enter comma-separated numbers")


def save_config(selected_calendars):
    """
    Save selected calendars to configuration file

    Args:
        selected_calendars: List of calendar dictionaries to save
    """
    config = {
        "calendars": [
            {
                "name": cal.get('summary', 'Unnamed'),
                "id": cal.get('id', ''),
                "enabled": True,
                "access_role": cal.get('accessRole', 'unknown')
            }
            for cal in selected_calendars
        ],
        "sync_settings": {
            "days_ahead": 30,
            "days_behind": 7,
            "max_results_per_calendar": 100
        },
        "create_settings": {
            "default_calendar_id": selected_calendars[0].get('id', 'primary') if selected_calendars else 'primary',
            "default_duration_minutes": 60,
            "allow_calendar_selection": True
        }
    }

    with open(CONFIG_FILE, 'w') as f:
        json.dump(config, f, indent=2)

    print(f"\n[CONFIG] ✓ Configuration saved to {CONFIG_FILE}")


def main():
    """Main setup wizard flow"""
    print("=" * 80)
    print("ROSIE GOOGLE CALENDAR SETUP WIZARD")
    print("=" * 80)
    print("\nThis wizard will help you connect ROSIE to your Google Calendar.")
    print("You'll be able to:")
    print("  - Read events from selected calendars")
    print("  - Create new events via voice commands")
    print("  - Access shared calendars you have permission to view")

    # Step 1: Authenticate
    print("\n" + "-" * 80)
    print("STEP 1: Authentication")
    print("-" * 80)

    creds = authenticate()
    if not creds:
        return

    # Build Calendar API service
    try:
        service = build('calendar', 'v3', credentials=creds)
    except Exception as e:
        print(f"[ERROR] Failed to build Calendar service: {e}")
        return

    # Step 2: List calendars
    print("\n" + "-" * 80)
    print("STEP 2: Calendar Selection")
    print("-" * 80)

    calendars = list_calendars(service)
    if not calendars:
        print("[ERROR] No calendars available. Exiting.")
        return

    # Step 3: Select calendars
    selected = select_calendars(calendars)
    if not selected:
        print("[ERROR] No calendars selected. Exiting.")
        return

    # Step 4: Save configuration
    print("\n" + "-" * 80)
    print("STEP 3: Save Configuration")
    print("-" * 80)

    save_config(selected)

    # Step 5: Summary and next steps
    print("\n" + "=" * 80)
    print("SETUP COMPLETE!")
    print("=" * 80)
    print("\nNext steps:")
    print(f"1. Run the sync script to fetch calendar events:")
    print(f"   python3 {ROSIE_DIR}/src/rosie_calendar_sync.py")
    print(f"\n2. Check that events were added to your knowledge base:")
    print(f"   cat {ROSIE_DIR}/knowledge_base/calendar_events.md")
    print(f"\n3. Set up automatic syncing with cron (optional):")
    print(f"   */15 * * * * cd {ROSIE_DIR}/src && python3 rosie_calendar_sync.py")
    print("\n" + "=" * 80)


if __name__ == '__main__':
    main()
