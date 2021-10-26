using Documenter, ErrorBudget
push!(LOAD_PATH,"../src/")
makedocs(sitename="My Documentation")

makedocs(
         sitename = "ErrorBudget.jl",
         modules  = [ErrorBudget],
         pages=[
                "Home" => "index.md"
               ])
deploydocs(;
    repo="github.com/YayeIrene/ErrorBudget.jl",
)
