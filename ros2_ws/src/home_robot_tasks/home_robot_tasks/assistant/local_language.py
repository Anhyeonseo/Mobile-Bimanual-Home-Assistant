"""Adapt language slots, and schedule inference outside the control lock.

The worker only returns data. The gateway thread persists and grounds results;
no inference completion can execute or approve a task. Cancelled or interrupted
jobs cannot be revived by a late reply. Failures never fall back silently.
"""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import re

from ..fetch import InvalidTask
from ..navigation import identifier
from .contracts import text
from .language import NOUNS, COLORS, _matches, _remove_aliases


def meaning_from_slots(item):
    target = item.get('target') or ''
    categories = [k for k, names in NOUNS.items() if _matches(target, names)]
    colors = [k for k, names in COLORS.items() if _matches(target, names)]
    issues = []
    mapping = {'target': 'object_category_required', 'destination': 'destination_required',
               'multiple_actions': 'single_task_required',
               'negated_action': 'negated_action_requires_clarification'}
    issues.extend(mapping.get(v, v) for v in item['unresolved'])
    if len(categories) != 1:
        issues.append('object_category_required' if not categories else 'multiple_object_categories')
    if len(colors) > 1:
        issues.append('ambiguous_color')
    residual = _remove_aliases(target, [v for names in (*NOUNS.values(), *COLORS.values()) for v in names])
    if residual.strip():
        issues.append('unsupported_object_constraint')
    # The current world store has no camera-relative selection or ownership
    # evidence. Preserve these slots in the job, and ask instead of dropping them.
    if item.get('location') or re.search(r'왼쪽|오른쪽|앞쪽|뒤쪽', item['text']):
        issues.append('relative_location_requires_observation')
    if item.get('excluded') or re.search(r'말고|아니고|아니라|제외|빼고', item['text']):
        issues.append('exclusion_requires_clarification')
    operation = {'pick': 'fetch_object', 'move': 'fetch_object', 'fetch': 'fetch_object',
                 'find': 'find_object'}.get(item['action'])
    return dict(operation=operation, category=categories[0] if len(categories) == 1 else None,
                attributes={'color': colors[0]} if len(colors) == 1 else {},
                source_phrase=item.get('source') or '', destination_phrase=item.get('destination') or '',
                issues=list(dict.fromkeys(issues)))


class LanguageJobs:
    """Methods run under gateway lock; worker never reads app/SQLite state."""
    def __init__(self, service, model):
        self.service, self.model = service, model
        self.pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='local-language')
        self.pending = {}
        for job in service.store.records('language'):
            if job['status'] == 'INTERPRETING':
                job.update(status='INTERRUPTED', error='server_restarted')
                self._save(job)

    def _save(self, job):
        self.service.store.save_record('language', job['request_id'], job)
        return deepcopy(job)

    def get(self, rid):
        job = self.service.store.record('language', identifier(rid, 'request_id'))
        if job is None:
            raise InvalidTask('unknown_language_request')
        return job

    def submit(self, utterance, rid):
        identifier(rid, 'request_id'); text(utterance, 'utterance', 512)
        previous = self.service.store.record('language', rid)
        if previous:
            if previous['utterance'] != utterance:
                raise InvalidTask('request_id_conflict')
            return previous
        if self.service.store.record('intent', rid):
            raise InvalidTask('request_id_conflict')
        if self.pending:
            raise InvalidTask('language_busy')
        if len(self.service.store.records('language')) >= 128:
            raise InvalidTask('intent_history_full')
        job = dict(request_id=rid, utterance=utterance, status='INTERPRETING',
                   context=self.service.context.to_dict(), created_s=self.service.clock(), hardware_commands=0)
        self._save(job)
        self.pending[rid] = self.pool.submit(self.model.extract, utterance, rid)
        return job

    def cancel(self, rid):
        job = self.get(rid)
        if job['status'] == 'INTERPRETING':
            job.update(status='CANCELLED')
            self._save(job)
            if rid in self.pending:
                self.pending[rid].cancel()
        return job

    def cancel_pending(self):
        for rid in list(self.pending):
            self.cancel(rid)

    def poll(self):
        for rid, future in list(self.pending.items()):
            if not future.done():
                continue
            del self.pending[rid]
            job = self.get(rid)
            if job['status'] != 'INTERPRETING':
                continue
            if job['context'] != self.service.context.to_dict():
                job.update(status='INTERRUPTED', error='context_changed')
                self._save(job)
                continue
            try:
                extraction = future.result()
                meaning = meaning_from_slots(extraction['extraction'])
                result = self.service.prepare(job['utterance'], rid, meaning=meaning,
                                              parser='local_language_v1')
                job.update(status='READY', result=result, inference=extraction)
            except Exception:
                # No raw server exception in the UI, and no rule-parser fallback.
                job.update(status='FAILED', error='local_language_failed')
            self._save(job)

    def close(self):
        self.cancel_pending()
        self.pool.shutdown(wait=False, cancel_futures=True)
