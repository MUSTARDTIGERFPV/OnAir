// First-run tour with Onny the TV.
#include "app.h"
#include "fixit.h"
#include "dect2/maidenhead.h"
#include "dect2/mode_tuning.h"
#include "dect2/updater.h"

// First-run tour with Onny the TV: a friendly guide that explains the window, plays the synthetic demo signal and helps pick a radio.
// Included by main.cpp (needs App, setFamily, refreshDevices, savePrefs, gForceTab).


struct WizStep {
    const char* text;
    int target;
    mascot::Mood mood;
    bool wave;
    const char* action;    // label of an extra button (nullptr: none); handled in wizAction()
    const char* tab;       // tab to bring up while this step is shown (nullptr: leave)
};

static const WizStep kWiz[] = {
    {N_("Hi, I'm Onny! I'm a little TV who lives in OnAir. Want a quick tour? It takes about a minute, and I'll show you everything with a demo signal, so you don't need any hardware yet."),
     TgNone, mascot::Excited, true, nullptr, nullptr},
    {N_("Up here is the switch for what kind of broadcast you want: DVB for most of the world's digital TV, ATSC for North America, and DAB for digital radio. OnAir has all three in one place."),
     TgSwitch, mascot::Happy, false, nullptr, nullptr},
    {N_("Let's see it in action! I'll start a pretend signal. It is a test transmitter running inside the app, so nothing needs to be plugged in."),
     TgToolbar, mascot::Excited, false, N_("Start the demo signal"), nullptr},
    {N_("See the wiggly line? That is the spectrum: how strong everything on the air is, from left to right. The tall bump in the middle is our channel. Below it, the waterfall shows the same thing slowly scrolling by in time."),
     TgMain, mascot::Happy, false, nullptr, "Overview"},
    {N_("And these four boxes show the dots the receiver sees. When the signal is good, the dots gather into tidy clusters. Messy dots mean a noisy signal. Right now it's a clean demo, so they should look neat!"),
     TgConst, mascot::Happy, false, nullptr, nullptr},
    {N_("Over here you'll find the channels once the receiver locks on. Click one to watch it. There's also a player with picture and sound, a clock, and options to record or stream what you're watching."),
     TgRight, mascot::Happy, false, nullptr, nullptr},
    {N_("Now for the real thing. Pick your radio here, choose a frequency, and set the gain. Gain is how much your radio boosts the signal: too little and it's lost, too much and it distorts. The Auto-tune button finds a good value for you."),
     TgToolbar, mascot::Thinking, false, N_("Stop the demo and look for my radio"), nullptr},
    {N_("Don't know where the channels are? The Scan tab searches for you. A good antenna, placed high and near a window, makes a bigger difference than anything else. I promise!"),
     TgMain, mascot::Happy, false, nullptr, "Scan"},
    {N_("That's the whole tour! You can share your channels with other devices from the Stream tab (Outputs), and bring me back any time with the Tour button at the top right. Happy watching!"),
     TgNone, mascot::Excited, true, nullptr, nullptr},
};
static const int kWizSteps = (int)(sizeof kWiz / sizeof *kWiz);

void wizFinish(App& a) {
    a.wizOpen = false;
    plat::prefs().setB("wizardDone", true);
    plat::prefs().flush();
}

void wizEnter(App& a, int step) {
    a.wizStep = std::max(0, std::min(kWizSteps - 1, step));
    a.wizStepT = ImGui::GetTime();
    a.wizAcked = false;
    if (kWiz[a.wizStep].tab) gForceTab = kWiz[a.wizStep].tab;
}

void wizAction(App& a, int step) {
    if (step == 2) {   // start the synthetic demo
        for (int i = 0; i < (int)a.devices.size(); i++) if (a.devices[i].kind == DeviceInfo::Synthetic) a.devIdx = i;
        setFamily(a, 0);
        a.stdMode = 2;                 // DVB-T: the synthetic DVB-T signal carries a real test programme
        a.tune.synth.dvbt = true; a.tune.synth.demoTv = true;
        if (a.engine.running()) a.engine.stop();
        a.wizStart = true;
        a.playReq = 1;                 // play the demo programme as soon as the receiver has found it
    } else if (step == 6) {   // back to real hardware
        if (a.engine.running()) a.engine.stop();
        refreshDevices(a);
        a.devIdx = std::max(0, std::min(a.devIdx, (int)a.devices.size() - 1));   // a radio unplugged since: the list is shorter now
        for (int i = 0; i < (int)a.devices.size(); i++) if (a.devices[i].isRadio()) { a.devIdx = i; break; }
    }
}

void wizard(App& a, ImVec2 disp) {
    if (!a.wizOpen) return;
    const double now = ImGui::GetTime();
    const WizStep& st = kWiz[a.wizStep];
    ImDrawList* fg = ImGui::GetForegroundDrawList();
    const ImU32 acc = ImGui::ColorConvertFloat4ToU32(pal::accent());

    // the part of the window the step talks about gets a pulsing frame
    ImVec2 tgt(disp.x * 0.5f, disp.y * 0.3f);
    bool havePt = false;
    if (st.target != TgNone && a.tgMax[st.target].x > a.tgMin[st.target].x) {
        const ImVec2 lo = a.tgMin[st.target], hi = a.tgMax[st.target];
        const float pulse = 0.55f + 0.45f * (float)std::sin(now * 4.0);
        fg->AddRectFilled(lo, hi, IM_COL32(115, 184, 209, 14), 4.f);
        fg->AddRect(ImVec2(lo.x - 2, lo.y - 2), ImVec2(hi.x + 2, hi.y + 2), (acc & 0x00FFFFFF) | ((ImU32)(255 * pulse) << 24), 4.f, 0, 2.f);
        tgt = ImVec2((lo.x + hi.x) * 0.5f, std::min(hi.y - 12.f, lo.y + 40.f));
        havePt = true;
    }

    // the panel sits in a bottom corner and moves to the other one when it would cover what it is pointing at
    const float u = gUi;
    const float W = std::min(780.f * u, disp.x - 24.f * u), H = 196.f * u;
    const bool right = havePt && tgt.x < disp.x * 0.5f && tgt.y > disp.y - H - 80.f * u;
    const float goalX = right ? disp.x - W - 12.f * u : 12.f * u;
    if (a.wizX < 0) a.wizX = goalX;
    a.wizX += (goalX - a.wizX) * std::min(1.f, ImGui::GetIO().DeltaTime * 8.f);
    const ImVec2 pos(a.wizX, disp.y - H - 12.f * u);

    // typewriter
    const std::string text = (a.wizStep == 6 && !a.engine.running())
        ? [&] {
              std::string n;
              for (auto& d : a.devices) if (d.isRadio()) { n = d.name; break; }
              if (!n.empty()) {
                  std::string g = tr("Good news: I can see \"%s\"!");
                  const size_t at = g.find("%s");
                  if (at != std::string::npos) g.replace(at, 2, n);
                  return std::string(tr(st.text)) + " " + g;
              }
              return std::string(tr(st.text)) + " " + tr("(I can't see a radio right now. Plug it in and press the button again.)")
#ifdef _WIN32
                     + " " + tr("On Windows a HackRF needs its WinUSB driver once: install it with Zadig, at zadig.akeo.ie.")
#endif
                  ;
          }()
        : std::string(tr(st.text));
    size_t shown = std::min(text.size(), (size_t)((now - a.wizStepT) * 75.0));
    while (shown < text.size() && ((unsigned char)text[shown] & 0xC0) == 0x80) shown++;   // never half a UTF-8 character
    const bool talking = shown < text.size();

    ImGui::SetNextWindowPos(pos);
    ImGui::SetNextWindowSize(ImVec2(W, H));
    ImGui::PushStyleColor(ImGuiCol_WindowBg, ImVec4(0.075f, 0.082f, 0.09f, 0.97f));
    ImGui::PushStyleColor(ImGuiCol_Border, pal::accent(0.8f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 10.f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.5f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
    ImGui::Begin("##onny", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoScrollbar);
    ImGui::PopStyleVar(3);
    ImGui::PopStyleColor(2);

    // Onny himself, in the left part of the panel (drawn on the foreground list so the antenna may leave the panel)
    mascot::Pose pose;
    pose.t = now; pose.talking = talking; pose.mood = st.mood; pose.wave = st.wave; pose.point = havePt; pose.target = tgt;
    mascot::draw(fg, ImVec2(pos.x + 100.f * u, pos.y + 98.f * u), 0.95f * u, pose, acc);

    // speech: a text area and the buttons
    const float tx = 200.f * u;
    ImGui::SetCursorPos(ImVec2(tx, 14 * u));
    ImGui::PushTextWrapPos(W - 18.f * u);
    ImGui::TextUnformatted(text.c_str(), text.c_str() + shown);
    ImGui::PopTextWrapPos();
    if (ImGui::IsWindowHovered() && ImGui::IsMouseClicked(0) && talking) a.wizStepT = now - 1000;   // click: show everything at once

    // progress dots
    {
        const ImVec2 w0 = ImGui::GetWindowPos();
        for (int i = 0; i < kWizSteps; i++) {
            const ImVec2 c(w0.x + tx + 6 * u + i * 14 * u, w0.y + H - 62 * u);
            if (i == a.wizStep) ImGui::GetWindowDrawList()->AddCircleFilled(c, 4.f * u, acc);
            else ImGui::GetWindowDrawList()->AddCircle(c, 3.5f * u, IM_COL32(90, 100, 108, 255), 0, 1.2f);
        }
    }
    ImGui::SetCursorPos(ImVec2(tx, H - 44 * u));
    if (a.wizStep > 0 && ImGui::Button(tr("Back"), ImVec2(70 * gUi, 0))) { wizEnter(a, a.wizStep - 1); }
    if (a.wizStep > 0) ImGui::SameLine();
    if (st.action) {
        if (ImGui::Button(tr(st.action))) { wizAction(a, a.wizStep); a.wizAcked = true; }
        ImGui::SameLine();
    }
    const bool last = a.wizStep == kWizSteps - 1;
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.34f, 0.40f, 1)); ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.27f, 0.45f, 0.53f, 1));
    if (ImGui::Button(tr(last ? "Finish" : "Next"), ImVec2(80 * gUi, 0))) { if (last) wizFinish(a); else wizEnter(a, a.wizStep + 1); }
    ImGui::PopStyleColor(2);
    if (!last) {
        ImGui::SameLine(0, 18 * gUi);
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
        if (ImGui::Button(tr("Skip tour"))) wizFinish(a);
        ImGui::PopStyleColor();
    }
    ImGui::End();
}


// ---- The setup wizard: language, radio, location, goals -> a recommended starting mode, tuned to a sensible default ----
namespace {

struct Region {
    const char* name;
    int tvFamily, tvStd;   // the family of the digital TV there (0 DVB, 1 ATSC, 4 ISDB-T), and the DVB standard (1 T2, 2 T)
    double fmMhz;          // a sensible FM broadcast frequency
    const char* tvId;      // when not null: the TV mode by id instead (China)
};
const Region kRegions[] = {
    {N_("Europe"), 0, 1, 100.0, nullptr},
    {N_("United Kingdom and Ireland"), 0, 1, 97.0, nullptr},
    {N_("North America"), 1, 0, 99.5, nullptr},
    {N_("Middle East"), 0, 1, 93.9, nullptr},
    {N_("Australia and New Zealand"), 0, 2, 101.1, nullptr},
    {N_("Japan"), 4, 0, 90.0, nullptr},
    {N_("China"), 0, 0, 100.0, "dtmb"},
    {N_("Elsewhere"), 0, 1, 100.0, nullptr},
};
constexpr int kNumRegions = (int)(sizeof kRegions / sizeof *kRegions);

struct Goal { const char* label; const char* modeId; };   // the first ticked one (in this order) picks the starting mode
const Goal kGoals[] = {
    {N_("FM / DAB radio"), "fm"},
    {N_("Television"), "tv"},
    {N_("Aircraft"), "adsb"},
    {N_("Ships"), "ais"},
    {N_("Satellites"), "meteor"},
    {N_("Amateur radio"), "hfdig"},
    {N_("Trunked and two-way radio"), "dmr"},
    {N_("Sensors and IoT"), "ism"},
    {N_("Just explore"), "specan"},
};
constexpr int kNumGoals = (int)(sizeof kGoals / sizeof *kGoals);

int pickGoal(unsigned g) { for (int i = 0; i < kNumGoals; i++) if (g & (1u << i)) return i; return kNumGoals - 1; }

// Switch to the recommended mode and tune its default.
void applyGoal(App& a, int goal, const Region& r) {
    const std::string id = kGoals[goal].modeId;
    if (id == "fm") { setFamily(a, 5); a.freqMhz = r.fmMhz; return; }
    if (id == "tv") {
        if (r.tvId) { if (const ModeTuning* mt = modeTuningById(r.tvId)) { setFamily(a, mt->stdMode - 2); a.freqMhz = mt->defMhz; return; } }
        setFamily(a, r.tvFamily);
        if (r.tvFamily == 0) a.stdMode = r.tvStd;
        return;
    }
    int n = 0;
    const SubMode* t = subModes(n);
    for (int i = 0; i < n; i++) if (id == t[i].id) { setMode(a, t[i].stdMode - 2, i + 1); a.freqMhz = t[i].defMhz; return; }
    if (const ModeTuning* mt = modeTuningById(id)) { setFamily(a, mt->stdMode - 2); a.freqMhz = mt->defMhz; }
}

std::string goalModeName(int goal, const Region& r) {
    const std::string id = kGoals[goal].modeId;
    if (id == "fm") { char b[48]; snprintf(b, sizeof b, " (%.1f MHz)", r.fmMhz); return std::string(tr("FM radio")) + b; }
    if (id == "tv") return r.tvId ? modeTuningById(r.tvId)->name : r.tvFamily == 1 ? "ATSC" : r.tvFamily == 4 ? "ISDB-T" : r.tvStd == 2 ? "DVB-T" : "DVB-T2";
    int n = 0;
    const SubMode* t = subModes(n);
    for (int i = 0; i < n; i++) if (id == t[i].id) return t[i].name;
    if (const ModeTuning* mt = modeTuningById(id)) return mt->name;
    return id;
}

bool parseLoc(const char* text, double& lat, double& lon) {
    const std::string s = text;
    if (s.empty()) return false;
    return dect2::fromMaidenhead(s, lat, lon) || dect2::parseCoordinates(s, lat, lon);
}

}

void setupWizardOpen(App& a) {
    a.setupOpen = true;
    a.setupStep = 0;
    a.setupRegion = (int)std::max(0L, std::min((long)kNumRegions - 1, plat::prefs().getI("setupRegion", 0)));
    a.setupGoals = (unsigned)plat::prefs().getI("setupGoals", 0);
    snprintf(a.setupLoc, sizeof a.setupLoc, "%s", plat::prefs().getS("setupLoc", "").c_str());
}

static void setupFinish(App& a, bool apply) {
    a.setupOpen = false;
    if (apply) applyGoal(a, pickGoal(a.setupGoals), kRegions[a.setupRegion]);
    plat::prefs().setB("setupDone", true);
    plat::prefs().setI("setupRegion", a.setupRegion);
    plat::prefs().setI("setupGoals", (long)a.setupGoals);
    plat::prefs().setS("setupLoc", a.setupLoc);
    double lat, lon;
    if (parseLoc(a.setupLoc, lat, lon)) { plat::prefs().setD("adsbLat", lat); plat::prefs().setD("adsbLon", lon); }
    savePrefs(a);
    plat::prefs().flush();
    if (!plat::prefs().getB("wizardDone", false)) { a.wizOpen = true; a.wizX = -1; a.wizStep = 0; a.wizStepT = ImGui::GetTime(); }   // first run: the tour comes next
}

void setupWizard(App& a, ImVec2 disp) {
    if (!a.setupOpen) return;
    const float u = gUi;
    ImGui::SetNextWindowPos(ImVec2(disp.x * 0.5f, disp.y * 0.45f), ImGuiCond_Always, ImVec2(0.5f, 0.5f));
    ImGui::SetNextWindowSize(ImVec2(std::min(620.f * u, disp.x - 24 * u), 0));
    ImGui::PushStyleColor(ImGuiCol_Border, pal::accent(0.8f));
    ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 1.5f);
    ImGui::Begin("##setup", nullptr, ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_AlwaysAutoResize);
    ImGui::PopStyleVar();
    ImGui::PopStyleColor();
    constexpr int kSteps = 5;
    ImGui::TextColored(pal::accent(), "%s  (%d / %d)", tr("Setup"), a.setupStep + 1, kSteps);
    ImGui::Separator();
    ImGui::Spacing();
    ImGui::PushTextWrapPos(560 * u);
    switch (a.setupStep) {
    case 0: {
        ImGui::TextUnformatted(tr("Which language do you want OnAir in?"));
        ImGui::Spacing();
        if (ImGui::RadioButton(tr("Same as the system"), a.uiLang.empty())) setUiLanguage(a, "");
        int n = 0;
        const i18n::Language* L = i18n::languages(n);
        for (int i = 0; i < n; i++) if (ImGui::RadioButton(L[i].name, !a.uiLang.empty() && a.uiLang == L[i].code)) setUiLanguage(a, L[i].code);
        break;
    }
    case 1: {
        ImGui::TextUnformatted(tr("Which radio do you have?"));
        ImGui::Spacing();
        int radios = 0;
        for (int i = 0; i < (int)a.devices.size(); i++) {
            if (!a.devices[i].isRadio()) continue;
            radios++;
            if (ImGui::RadioButton((a.devices[i].name + "##r" + std::to_string(i)).c_str(), a.devIdx == i)) a.devIdx = i;
        }
        if (!radios) {
            ImGui::TextColored(pal::warnAmber(), "%s", tr("No radio found"));
            if (const fixit::Entry* e = fixit::byId("no_radio")) {
                ImGui::TextUnformatted(tr(e->why));
                int k = 1;
                for (const char* st : fixit::stepsFor(*e, fixit::currentOs())) { ImGui::Text("%d.", k++); ImGui::SameLine(); ImGui::TextUnformatted(tr(st)); }
                bool first = true;
                for (int b = 0; b < 3 && e->buttons[b].label; b++) {
                    if (!(e->buttons[b].os & fixit::currentOs())) continue;
                    if (!first) ImGui::SameLine();
                    first = false;
                    if (ImGui::Button(tr(e->buttons[b].label))) {
                        if (e->buttons[b].act == fixit::Act::OpenUrl) dect2::openUrl(e->buttons[b].arg);
                        else if (e->buttons[b].act == fixit::Act::CopyText) ImGui::SetClipboardText(e->buttons[b].arg);
                    }
                }
            }
            ImGui::Spacing();
            ImGui::TextUnformatted(tr("You can go on without one: the synthetic signal and IQ files work with no hardware."));
            ImGui::TextUnformatted(tr("A radio on the network is not found by looking: add it by its address."));
        }
        ImGui::Spacing();
        if (ImGui::Button(tr("Look again"))) refreshDevices(a);
        // a radio on the network is never found by looking: it is added by its address (a PlutoSDR by IP, rtl_tcp, SpyServer, SDR++ server, ...)
        ImGui::SameLine();
        if (ImGui::Button(tr("Add a network radio..."))) { a.netAddOpen = true; a.netAddErr.clear(); }
        if (ImGui::IsItemHovered()) ImGui::SetTooltip("%s", tr("A radio on another computer or on the network: a PlutoSDR by IP, an rtl_tcp server, a SpyServer, an SDR++ server, SoapyRemote, a Hermes Lite 2 or a Red Pitaya"));
        break;
    }
    case 2: {
        ImGui::TextUnformatted(tr("Where are you? This picks the right TV standard and FM band."));
        ImGui::Spacing();
        ImGui::SetNextItemWidth(260 * u);
        if (ImGui::BeginCombo("##region", tr(kRegions[a.setupRegion].name))) {
            for (int i = 0; i < kNumRegions; i++) if (ImGui::Selectable(tr(kRegions[i].name), i == a.setupRegion)) a.setupRegion = i;
            ImGui::EndCombo();
        }
        ImGui::Spacing();
        ImGui::TextUnformatted(tr("Optional: your position, as a Maidenhead locator (KO24pq) or coordinates (54.68, 25.28). It is kept on this computer and used for maps and satellite passes."));
        ImGui::SetNextItemWidth(260 * u);
        ImGui::InputText("##loc", a.setupLoc, sizeof a.setupLoc);
        double lat, lon;
        if (a.setupLoc[0]) {
            if (parseLoc(a.setupLoc, lat, lon)) ImGui::TextDisabled("%.3f, %.3f  (%s)", lat, lon, dect2::toMaidenhead(lat, lon, 6).c_str());
            else ImGui::TextColored(pal::warnAmber(), "%s", tr("Not a locator or coordinates"));
        }
        break;
    }
    case 3: {
        ImGui::TextUnformatted(tr("What do you want to receive? Tick anything that interests you."));
        ImGui::Spacing();
        for (int i = 0; i < kNumGoals; i++) {
            bool on = (a.setupGoals >> i) & 1u;
            if (ImGui::Checkbox(tr(kGoals[i].label), &on)) a.setupGoals = on ? (a.setupGoals | (1u << i)) : (a.setupGoals & ~(1u << i));
        }
        break;
    }
    default: {
        const int g = pickGoal(a.setupGoals);
        ImGui::TextUnformatted(tr("All set. OnAir will start here:"));
        ImGui::Spacing();
        ImGui::TextColored(pal::accent(), "%s", goalModeName(g, kRegions[a.setupRegion]).c_str());
        ImGui::Spacing();
        ImGui::TextUnformatted(tr("You can change the mode any time, and run this setup again from Help > Setup wizard."));
        break;
    }
    }
    ImGui::PopTextWrapPos();
    ImGui::Spacing();
    ImGui::Separator();
    if (a.setupStep > 0 && ImGui::Button(tr("Back"), ImVec2(70 * u, 0))) a.setupStep--;
    if (a.setupStep > 0) ImGui::SameLine();
    const bool last = a.setupStep == kSteps - 1;
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.20f, 0.34f, 0.40f, 1)); ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.27f, 0.45f, 0.53f, 1));
    if (ImGui::Button(tr(last ? "Finish" : "Next"), ImVec2(90 * u, 0))) { if (last) setupFinish(a, true); else a.setupStep++; }
    ImGui::PopStyleColor(2);
    ImGui::SameLine(0, 18 * u);
    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0, 0, 0, 0));
    if (ImGui::Button(tr("Skip setup"))) setupFinish(a, false);
    ImGui::PopStyleColor();
    ImGui::End();
}
