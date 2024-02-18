from datetime import datetime


from google.oauth2.credentials import Credentials
from google_auth_oauthlib.flow import InstalledAppFlow
from google.auth.transport.requests import Request
from googleapiclient.discovery import build
import os.path
import pickle
from datetime import datetime, timedelta
import pytz



def Showing_Events_Calender():
    # If modifying these SCOPES, delete the file token.pickle.
    SCOPES = ['https://www.googleapis.com/auth/calendar.readonly']

    creds = None
    # The file token.pickle stores the user's access and refresh tokens, and is
    # created automatically when the authorization flow completes for the first time.
    if os.path.exists('token.pickle'):
        with open('token.pickle', 'rb') as token:
            creds = pickle.load(token)
    # If there are no (valid) credentials available, let the user log in.
    if not creds or not creds.valid:
        if creds and creds.expired and creds.refresh_token:
            creds.refresh(Request())
        else:
            flow = InstalledAppFlow.from_client_secrets_file(
                'credentials.json', SCOPES)
            creds = flow.run_local_server(port=0)
        # Save the credentials for the next run
        with open('token.pickle', 'wb') as token:
            pickle.dump(creds, token)

    service = build('calendar', 'v3', credentials=creds)

    # Get today's date in YYYY-MM-DD format
    today_date = datetime.now().strftime("%Y-%m-%d")

    date_input = today_date

    # Ask for the date input
    # date_input = input(f"Enter the date in YYYY-MM-DD format or press Enter to use today's date ({today_date}): ")

    # if not date_input:
    #     date_input = today_date
    try:
        #date = datetime.strptime(date_input, "%Y-%m-%d").astimezone(pytz.utc)
        date = datetime.strptime(date_input, "%Y-%m-%d").astimezone()
    except ValueError:
        print("Invalid date format. Please use YYYY-MM-DD format.")
        return

    start_of_day = date.replace(hour=0, minute=0, second=0, microsecond=0)
    end_of_day = start_of_day + timedelta(days=1)

    events_result = service.events().list(calendarId='primary', timeMin=start_of_day.isoformat(),
                                          timeMax=end_of_day.isoformat(), singleEvents=True,
                                          orderBy='startTime').execute()
    events = events_result.get('items', [])
    #print(events)

    if not events:
        print('No events found on this day.')
    else:
        #print(f"Events on {date_input}:")
        inter = 1
        for event in events:
            start = event['start'].get('dateTime', event['start'].get('date'))
            #print(event)
            if 'summary' in event:
                Title = event['summary']
            else:
                Title = "without title"

            # Extracting start and end times from the provided data
            start_time = event['start']['dateTime']
            end_time = event['end']['dateTime']
            
            # Formatting the times
            start_time_formatted = start_time.split('T')[1][:5]
            end_time_formatted = end_time.split('T')[1][:5]
            
            formatted_time_range = f"From {start_time_formatted} to {end_time_formatted}"

            if 'location' in event:
                Location = event['location']
            else:
                Location = "a not specified location"

            if 'attendees' in event:
                attendees = event.get('attendees', [])
                attendee_emails = [attendee['email'] for attendee in attendees]
            else:
                attendee_emails = "No attendees"
            
            booked_event = "Your meeting " + Title + " " + formatted_time_range + " at " + Location + " has been booked successfully."
            

            # print(" ")
            # print("Event %d: "%inter)
            # print("Title: ", Title)
            # print("Time: ", formatted_time_range)
            # print("Location: ", Location)
            # print("attendees: ", attendee_emails)
            inter = inter +1
            #print(event['start'].get('dateTime', event['start']))
            #print(f"Start Time: {start}, Event: {event['summary']}")
            return booked_event

if __name__ == '__main__':
    Showing_Events_Calender()
