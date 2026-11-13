// Tagged templates keep view markup readable without a rendering dependency.
// Dynamic user content must still pass through escapeHTML before interpolation.
export function html(parts, ...values) {
  return parts.reduce(
    (result, part, index) => result + part + (index < values.length ? String(values[index]) : ''),
    '',
  );
}
