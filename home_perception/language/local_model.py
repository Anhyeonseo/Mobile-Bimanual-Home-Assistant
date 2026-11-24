"""Loopback-only llama.cpp client; no tools, credentials or robot access."""
import json
import time
from urllib.parse import urlsplit
from urllib.request import Request, build_opener, ProxyHandler, HTTPRedirectHandler

from .task_parser import PROMPT, parse_reply

SLOT_SCHEMA = {'type': 'object', 'additionalProperties': False,
               'properties': {**{key: {'type': ['string', 'null']} for key in
                                  ('target', 'source', 'destination')},
                              'excluded': {'type': 'array', 'items': {'type': 'string'}, 'maxItems': 8}},
               'required': ['target', 'source', 'destination', 'excluded']}


class NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, *args, **kwargs):
        raise ValueError('local model redirects are not allowed')


class LocalLanguageModel:
    def __init__(self, endpoint='http://127.0.0.1:18767', *, timeout_s=90):
        url = urlsplit(endpoint)
        if (url.scheme != 'http' or url.hostname not in {'127.0.0.1', '::1'} or
                url.username or url.password or url.path not in {'', '/'} or url.query or url.fragment):
            raise ValueError('numeric loopback model endpoint required')
        if not 1 <= timeout_s <= 120:
            raise ValueError('model timeout must be 1..120 seconds')
        self.endpoint, self.timeout_s = endpoint.rstrip('/'), timeout_s
        self.opener = build_opener(ProxyHandler({}), NoRedirect())

    def extract(self, utterance, request_id, user_id='local-user'):
        if not isinstance(utterance, str) or not 0 < len(utterance.strip()) <= 512:
            raise ValueError('bounded utterance required')
        started = time.monotonic()
        system = PROMPT.removesuffix('요청: ') + '\nJSON만 출력한다. /no_think'
        reply, data = self.complete_json(system, utterance, 'request_slots', SLOT_SCHEMA)
        item = parse_reply(reply, request_id, user_id, utterance)
        return {'extraction': item.to_dict(), 'reply': reply,
                'model': data.get('model', 'unknown'), 'backend': 'llama.cpp_loopback',
                'latency_s': round(time.monotonic()-started, 4), 'usage': data.get('usage', {})}

    def complete_json(self, system, utterance, name, schema):
        payload = dict(model='local-qwen', messages=[
            {'role': 'system', 'content': system}, {'role': 'user', 'content': utterance}],
            temperature=0, seed=42, max_tokens=256, stream=False,
            chat_template_kwargs={'enable_thinking': False},
            response_format={'type': 'json_schema', 'json_schema': {
                'name': name, 'strict': True, 'schema': schema}})
        request = Request(self.endpoint+'/v1/chat/completions',
                          data=json.dumps(payload, ensure_ascii=False).encode(),
                          headers={'Content-Type': 'application/json'})
        with self.opener.open(request, timeout=self.timeout_s) as response:
            raw = response.read(65537)
        if len(raw) > 65536:
            raise ValueError('model response too large')
        data = json.loads(raw)
        choice = data['choices'][0]
        if choice.get('finish_reason') != 'stop':
            raise ValueError('model reply did not finish')
        return choice['message']['content'], data
