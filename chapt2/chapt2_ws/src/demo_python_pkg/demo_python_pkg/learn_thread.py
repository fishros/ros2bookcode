import threading
import requests

class Download:
    def download(self, url, callback):
        print(f'线程:{threading.get_ident()} 开始下载：{url}')
        response = requests.get(url)
        response.encoding = 'utf-8'
        callback(url, response.text)

    def start_download(self, url, callback):
        thread = threading.Thread(target=self.download, args=(url, callback))
        thread.start()

def download_finish_callback(url, result):
    print(f'{url}下载完成,共：{len(result)}字,内容为：{result[:5]}...')

def main():
    d = Download()
    d.start_download('http://localhost:8000/novel1.txt', download_finish_callback)
    d.start_download('http://localhost:8000/novel2.txt', download_finish_callback)
    d.start_download('http://localhost:8000/novel3.txt', download_finish_callback)
