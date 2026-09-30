/* GA4 configuration and semantic events. No navigation interception. */
(function () {
  "use strict";
  var script = document.currentScript;
  var id = script && script.dataset.measurementId;
  if (!/^G-[A-Z0-9]+$/.test(id || "") || window.academicAnalytics) return;
  var granted = script.dataset.requireConsent !== "true";
  var loaded = false;
  window.dataLayer = window.dataLayer || [];
  window.gtag = window.gtag || function () { window.dataLayer.push(arguments); };
  function consent(command) {
    window.gtag("consent", command, {
      analytics_storage: granted ? "granted" : "denied",
      ad_storage: "denied", ad_user_data: "denied", ad_personalization: "denied"
    });
  }
  consent("default");
  function cleanUrl(value) {
    try {
      var url = new URL(value);
      return /^https?:$/.test(url.protocol) ? url.origin + url.pathname : "";
    } catch (_) { return ""; }
  }
  function load() {
    if (!granted || loaded) return;
    loaded = true;
    var config = {
      page_location: cleanUrl(window.location.href),
      page_referrer: cleanUrl(document.referrer),
      allow_google_signals: false, allow_ad_personalization_signals: false
    };
    // Only documented campaign labels survive; arbitrary query strings/fragments do not.
    var query = new URL(window.location.href).searchParams;
    ["source", "medium", "campaign", "content", "term"].forEach(function (name) {
      var value = query.get("utm_" + name);
      if (value && /^[a-z0-9_]{1,64}$/.test(value))
        config[name === "campaign" ? "campaign_name" : "campaign_" + name] = value;
    });
    if (script.dataset.debug === "true") config.debug_mode = true;
    window.gtag("js", new Date());
    window.gtag("config", id, config);
    var tag = document.createElement("script");
    tag.async = true;
    tag.src = "https://www.googletagmanager.com/gtag/js?id=" + encodeURIComponent(id);
    document.head.appendChild(tag);
  }
  // A consent UI can call this after initialization; no choice is persisted here.
  window.academicAnalytics = {
    setConsent: function (allow) {
      granted = allow === true;
      consent("update");
      load();
    }
  };
  var schemas = {
    resource_click: {
      resource_type: /^(publication|project|software|dataset|profile)$/,
      resource_id: /^[a-z0-9][a-z0-9_+.-]{0,63}$/,
      resource_action: /^(paper|code|project_page|dataset|demo|video|slides|supplementary|visit)$/,
      research_area: /^[a-z0-9][a-z0-9_-]{0,63}$/
    },
    cv_download: { cv_variant: /^(academic|full|jobmarket|editorial)$/, language: /^(en|zh)$/ },
    contact_click: { contact_method: /^(email|institution_profile|lab_page)$/ },
    language_switch: { from_language: /^(en|zh)$/, to_language: /^(en|zh)$/ }
  };
  function track(event) {
    if (!granted || event.defaultPrevented || (event.type === "auxclick" && event.button !== 1)) return;
    var target = event.target;
    var link = target && target.closest && target.closest("a[data-ga-event]");
    if (!link) return;
    var name = link.dataset.gaEvent;
    var schema = Object.prototype.hasOwnProperty.call(schemas, name) && schemas[name];
    if (!schema || typeof window.gtag !== "function") return;
    var params = {};
    var valid = Object.keys(schema).every(function (key) {
      var value = link.getAttribute("data-" + key.replace(/_/g, "-"));
      if (!value && key === "research_area") return true;
      if (!schema[key].test(value || "")) return false;
      params[key] = value;
      return true;
    });
    if (valid) {
      try { window.gtag("event", name, params); } catch (_) { /* Navigation remains native. */ }
    }
  }
  document.addEventListener("click", track);
  document.addEventListener("auxclick", track);
  load();
}());
