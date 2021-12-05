cd(@__DIR__)
using Pkg
Pkg.activate(".")

includet("PortAudioSynth.jl")


using GLMakie


let
    f = Figure(fontsize = 24)
    display(f)

    running = Observable(false)
    b = Button(f[1, 1], label = @lift($running ? "Stop" : "Start"), tellwidth = false)

    freqs = 220 .* (1.0, 1.25, 1.5)
     sines = [PortAudioSynth.SineGenerator(44100, 0.0, freq, 0.3, 0.0) for freq in freqs]

    lsg = labelslidergrid!(f, "Sine Generator " .* ["1", "2", "3"], Ref(120:880), formats = "{}Hz")
    f[0, 1] = lsg.layout

    for (i, s) in enumerate(lsg.sliders)
        set_close_to!(s, freqs[i])
        on(s.value) do val
            sines[i].frequency_hz = val
        end
    end

    on(b.clicks) do c
        running[] = !running[]
        if running[]
            @async PortAudioSynth.run() do
                if !running[]
                    return nothing
                end

                sum(PortAudioSynth.next_sample!(s) for s in sines)
            end
        end
    end
end