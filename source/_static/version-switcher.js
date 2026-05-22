(function () {
  function decodeEntry(item) {
    if (!item) return null;
    if (typeof item === "string") return { name: item, label: item };
    if (typeof item.name === "string") {
      return {
        name: item.name,
        label: typeof item.label === "string" ? item.label : item.name,
      };
    }
    if (item.__jsonclass__ && Array.isArray(item.__jsonclass__)) {
      var payload = item.__jsonclass__[1];
      if (Array.isArray(payload) && typeof payload[0] === "string") {
        return { name: payload[0], label: payload[0] };
      }
    }
    return null;
  }

  function candidateVersionsUrls(pathname) {
    var clean = pathname.split("?")[0].split("#")[0];
    var parts = clean.split("/").filter(Boolean);
    var candidates = [];
    for (var i = parts.length; i >= 0; i--) {
      candidates.push("/" + parts.slice(0, i).join("/") + (i ? "/" : "") + "versions.json");
    }
    return Array.from(new Set(candidates));
  }

  function currentVersionFromPath(entries) {
    var names = entries.map(function (e) { return e.name; });
    var parts = window.location.pathname.split("/").filter(Boolean);
    for (var i = 0; i < parts.length; i++) {
      if (names.indexOf(parts[i]) !== -1) return parts[i];
    }
    return null;
  }

  var CACHE_KEY = "polyversion:versions";

  function buildSwitcher(rootUrl, entries) {
    var container = document.getElementById("version-switcher-container");
    var select = document.getElementById("version-switcher");
    if (!container || !select || !entries.length) return;

    var current = currentVersionFromPath(entries);
    var root = rootUrl.replace(/\/versions\.json$/, "");

    var ordered = entries.slice();
    if (!ordered.some(function (e) { return e.name === "latest"; })) {
      ordered.unshift({ name: "latest", label: "latest" });
    }

    // Replace the server-rendered placeholder option with the real entries.
    select.innerHTML = "";

    ordered.forEach(function (entry) {
      var opt = document.createElement("option");
      opt.value = root + "/" + entry.name + "/";
      opt.textContent = entry.label;
      if (entry.name === current) opt.selected = true;
      select.appendChild(opt);
    });

    container.hidden = false;
  }

  function renderFromCache() {
    try {
      var raw = sessionStorage.getItem(CACHE_KEY);
      if (!raw) return false;
      var cached = JSON.parse(raw);
      if (!cached || !cached.url || !Array.isArray(cached.entries) || !cached.entries.length) return false;
      buildSwitcher(cached.url, cached.entries);
      return true;
    } catch (_e) {
      return false;
    }
  }

  async function init() {
    // Synchronously populate from cache so the switcher does not flicker on navigation.
    var hadCache = renderFromCache();

    var urls = candidateVersionsUrls(window.location.pathname);
    for (var i = 0; i < urls.length; i++) {
      try {
        var res = await fetch(urls[i], { cache: "no-store" });
        if (!res.ok) continue;
        var data = await res.json();
        if (!Array.isArray(data)) continue;

        var entries = data.map(decodeEntry).filter(Boolean);
        if (!entries.length) continue;

        buildSwitcher(urls[i], entries);
        try {
          sessionStorage.setItem(CACHE_KEY, JSON.stringify({ url: urls[i], entries: entries }));
        } catch (_e) {}
        return;
      } catch (_e) {
        // Try next location.
      }
    }

    // If nothing was rendered (no cache, no successful fetch) keep the static placeholder visible.
    if (!hadCache) {
      var container = document.getElementById("version-switcher-container");
      if (container) container.hidden = false;
    }
  }

  document.addEventListener("DOMContentLoaded", init);
})();
