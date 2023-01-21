
import sys
import time
from datetime import datetime
import urllib.parse
import os
import requests
from requests.auth import HTTPBasicAuth
from bs4 import BeautifulSoup
import falconspy

DOWNLOAD_FOLDER = falconspy.FALCONS_PATH + '/matchLogs2020/videos/'

def get_videos_from_day(year, month, day):
    base_url = 'https://nexus.falcons-robocup.nl/service/rest/repository/browse/topcam/field_videos/'
    url = base_url + str(year) + '/' + str(month) + '/' + str(day)

    page = requests.get(url, auth=HTTPBasicAuth('topcam', 'topcam'))
    soup = BeautifulSoup(page.text, 'html.parser')

    videos = []
    for link in soup.find_all('a'):
        link_href = link.get('href')
        if link_href.endswith('mp4'):
            videos.append(link_href)

    return videos


def download_videos(video_list, year, month, day):
    try:
        os.mkdirs(DOWNLOAD_FOLDER)
    except:
        pass

    existing_files = os.listdir(DOWNLOAD_FOLDER)
    # remove extension and fractional seconds from files
    existing_files = [f.split('.')[0] for f in existing_files]

    local_files = []    
    for video_url in video_list:
        try:
            local_filename = video_url.split('/')[-1]
            local_filename = urllib.parse.unquote(local_filename)
            
            hour = int(local_filename.split(':')[0])
            minute = int(local_filename.split(':')[1])
            full_seconds = local_filename.split(':')[2]
            second = int(full_seconds.split('.')[0])
            microsecond = int(full_seconds.split('.')[1])

            dt = datetime(year, month, day, hour, minute, second, microsecond)
            unix_time = time.mktime(dt.timetuple()) + (dt.microsecond/1e6)

            unix_seconds_str = str(int(time.mktime(dt.timetuple())))

            if unix_seconds_str not in existing_files:
                local_filename = DOWNLOAD_FOLDER + str(unix_time) + '.mp4'
                local_files.append(local_filename)
                
                print('Downloading ' + video_url)
                with requests.get(video_url, stream=True, auth=('topcam', 'topcam')) as r:
                    r.raise_for_status()
                    with open(local_filename, 'wb') as f:
                        for chunk in r.iter_content(chunk_size=8192): 
                            f.write(chunk)
        except Exception as e:
            print(e)

    return local_files

def sync_day(year, month, day):
    videos = get_videos_from_day(year, month, day)
    print(videos)
    download_videos(videos, year, month, day)


if __name__ == '__main__':

    now = datetime.now()
    year = now.year
    month = now.month
    day = now.day

    if len(sys.argv) >= 4:
        year = int(sys.argv[1])
        month = int(sys.argv[2])
        day = int(sys.argv[3])
    
    sync_day(year, month, day)